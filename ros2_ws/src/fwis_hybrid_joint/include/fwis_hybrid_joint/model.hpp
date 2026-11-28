#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "fwis_planning_core/collision.hpp"

namespace fwis_hybrid_joint {
using fwis_planning_core::Pose2d;
using fwis_planning_core::GridSnapshot;
using fwis_planning_core::Footprint;
using CargoPolygon = std::vector<fwis_planning_core::Point2d>;
using Clock = std::chrono::steady_clock;
enum class Mode { A, O, S };
enum class Kind { Motion, Prepare, Hold };
using Headings = std::array<double, 3>;
using Gears = std::array<int, 3>;
using Quintic = std::array<double, 6>; // Bernstein coefficients at degree 5
struct Piece { Quintic x{}, y{}, yaw{}; }; // unwrapped formation-reference yaw
// Shortest to longest normalized ramp of the symmetric cubic-velocity cruise law.
inline constexpr std::array<double, 4> cruise_ramps{1./64, 1./32, 1./16, 1./8};
inline constexpr bool supported_cruise_ramp(double width) {
  for (double valid : cruise_ramps) if (width == valid) return true;
  return false;
}
struct Phase {
  Kind kind{Kind::Motion};
  Mode mode{Mode::A};
  Gears gear{1,1,1};
  double duration{};
  std::vector<Piece> pieces; // equal spans of normalized station
  Headings heading0{}, heading1{}; // S endpoints or O/Hold constant headings
  double cruise_ramp_fraction{0.0}; // 0 uses quintic time; positive supported widths are for A/O cruise
};
struct Boundary {
  Pose2d reference{};
  Headings heading{};
  bool constrain_reference_yaw{true};
  std::array<bool,3> constrain_heading{true,true,true};
};
struct Limits {
  double radius{0.90}, speed{0.5}, acceleration{0.45}, jerk{2.0}, lateral_acceleration{2.0};
  double spin_speed{0.55}, spin_acceleration{0.7}, spin_jerk{3.0};
  double yaw_rate{2.5}, yaw_acceleration{0.0}, yaw_jerk{0.0};
  double wheelbase{0.494}, track{0.364}, wheel_speed{3.0};
  double steer_angle{1.5707963267948966}, steer_rate{0.8}, steer_acceleration{1.5};
  double prepare_time{0.7}; // protocol lower bound, plus wheel travel as required
  double min_motion_duration{0.30}; // dwell in a moving phase, seconds
  double cargo_center_speed{0.0}; // optional Cartesian limit at the cargo center
  double high_steer_angle{0.3490658503988659};
  double high_steer_speed{0.0}; // optional body speed limit above high_steer_angle
};
struct Request {
  std::shared_ptr<const GridSnapshot> grid;
  std::vector<Footprint> vehicles; // size 1 at offset (0,0), or 2–3 rigidly positioned mounts
  std::optional<Footprint> cargo;
  Boundary start, goal;
  Limits limits;
  bool allow_reverse{true};
  double task_duration_cap{300.0};
  bool align_terminal_chassis{false};
};
struct Plan { std::vector<Phase> phases; };
struct Body {
  Pose2d pose{};
  double vx{},vy{},wz{},curvature{},oblique_angle{};
};
struct Sample {
  Pose2d reference{};
  std::array<Body,3> bodies{};
  Mode mode{Mode::A};
  Kind kind{Kind::Motion};
};
// A single rest-to-rest station clock spans the entire phase and all its geometric pieces.
// During A, body yaw is the tangent to the moving mount, shifted by pi for reverse.
// During O body yaw is fixed; S holds F, interpolates body headings by normalized station.
// Single A ignores reference yaw polynomial; reference yaw equals its body's tangent heading.
// A stationary single S keeps reference XY only and follows the spinning body in reference yaw.
Sample evaluate_phase(const Request&, const Phase&, double seconds);
Sample evaluate(const Request&, const Plan&, double seconds);
double duration(const Plan&);
struct Audit {
  bool executable{};
  std::string reason;
  std::size_t phase{}, vehicle{};
  double u0{}, u1{};
};
using Cancel = std::function<bool()>;
Audit certify(const Request&, const Plan&, Clock::time_point deadline, const Cancel& cancel = {});
// The supplied local outline replaces the rectangular cargo in map queries.
// Rectangular metadata still locates the cargo centre for the speed limit.
Audit certify_polygon(const Request&, const Plan&, const CargoPolygon&, Clock::time_point deadline,
                      const Cancel& cancel = {});
// Dimensionless policy costs for a separately certified model. Soft clearance
// and effort are deterministic quadratures, never collision/safety tests.
struct PolicyScore {
  int auxiliary_blocks{};
  double auxiliary_cost{},execution_cost{},reference_length{},oblique_length{},spin_angle{};
  double steering_effort{},jerk_effort{},clearance_effort{};
  bool better_than(const PolicyScore&) const;
};
PolicyScore score_policy(const Request&,const Plan&,Clock::time_point deadline,const Cancel& cancel={});

struct Attempt {
  std::string proposal,status;
  double nlp_build_s{},nlp_solve_s{},certify_s{},wall_s{};
};
struct Result {
  double seed_seconds{}, edit_seconds{};
  std::vector<Attempt> attempts;
  std::optional<Plan> plan;
  std::string status, reason;
  std::vector<std::string> trace;
  double first_certified_s{}, first_policy_compliant_s{};
  std::string terminal_alignment{"not_requested"};
};
struct SolveOptions {
  std::vector<Mode> initial_sequence; // optional cold seed for structural ablations
  bool enable_edits{true};
  bool movable_switches{true};
};
Result solve(const Request&, Clock::time_point deadline, const Cancel& cancel = {},
             const SolveOptions& options = {});
// Single vehicle driving to a stationary cargo mount. The complete path,
// including final chassis alignment, is certified against the input request.
Result solve_docking(const Request&, Clock::time_point deadline);
} // namespace fwis_hybrid_joint
