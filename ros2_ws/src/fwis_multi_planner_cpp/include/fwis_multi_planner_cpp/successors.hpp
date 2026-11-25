#pragma once

#include "fwis_planning_core/geometry.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace fwis_multi_planner_cpp
{
using fwis_planning_core::Point2d;

enum class Primitive {oblique, straight, arc, spin};
enum class Refusal {none, reverse_required, alignment_required, arc_domain};

struct TranslationSeed
{
  std::string label;
  Point2d displacement;
};

struct ArcSeed
{
  std::string label;
  Point2d pivot;
  double delta_rad;
};

struct Candidate
{
  Primitive primitive;
  std::string label;
  Point2d control;  // world displacement (A straight/O), cargo pivot (A arc)
  double delta_rad;
  std::vector<int> directions;
  std::vector<double> angles;  // body oblique angles or S wrapped increments
  Refusal refusal = Refusal::none;
  std::size_t failed_vehicle = 0;
  double bearing_rad = 0.0;
};

// Enumerate exact FR03 branches in O, A straight, A arc, deduplicated S
// order. Vehicle arrays are in fixed formation ID order. Inputs are a
// validated model and the configured finite geometry seed library.
std::vector<Candidate> generate_candidates(
  double formation_yaw_rad, const std::vector<Point2d> & offsets,
  const std::vector<double> & body_headings, const std::vector<TranslationSeed> & oblique,
  const std::vector<TranslationSeed> & straight, const std::vector<ArcSeed> & arcs,
  double minimum_radius_m, bool allow_reverse, bool prepare_spins);

// Build standard eight cargo-frame O directions, the target-directed A/O
// seed, and the configured fixed/target-cropped A arc library. The optional
// target step is already capped to the configured distance by the goal adapter.
std::vector<Candidate> generate_lattice(
  double formation_yaw_rad, const std::vector<Point2d> & offsets,
  const std::vector<double> & headings, double step_m, std::optional<Point2d> target_step,
  const std::vector<Point2d> & pivots, double arc_step_rad, std::optional<double> target_yaw_rad,
  double minimum_radius_m, bool allow_reverse);

std::optional<std::vector<double>> spin_increments(
  const std::vector<double> & start, const std::vector<double> & target);

struct GoalArcBranch
{
  std::vector<int> directions;
  std::vector<double> target_headings;
  double maximum_spin_rad;
  double total_spin_rad;
};

std::optional<std::vector<GoalArcBranch>> goal_arc_branches(
  double formation_yaw_rad, const std::vector<Point2d> & offsets,
  const std::vector<double> & headings, Point2d pivot, double delta_rad,
  double minimum_radius_m, bool allow_reverse);

struct WheelSetup
{
  char mode;  // A, O or S
  std::array<double, 4> angles;
};

struct SearchLimits
{
  double mode_setup_s;
  double mode_switch_s;
  double steer_rate_radps;
  double steer_accel_radps2;
  double speed_mps;
  double yaw_rate_radps;
  bool fixed_setup;
};

// Proxy ordering cost only. Independent FR02 retiming remains authoritative.
std::array<double, 2> edge_cost(
  const std::vector<WheelSetup> & before, const std::vector<WheelSetup> & after,
  const std::vector<double> & center_travel_m, const std::vector<double> & yaw_travel_rad,
  SearchLimits limits);
}  // namespace fwis_multi_planner_cpp
