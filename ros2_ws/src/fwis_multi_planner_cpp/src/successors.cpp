#include "fwis_multi_planner_cpp/successors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace fwis_multi_planner_cpp
{
namespace
{
using fwis_planning_core::wrap_angle;
constexpr double pi = 3.1415926535897932384626433832795;
constexpr double heading_tolerance = 1e-9;
constexpr double pivot_tolerance = 1e-9;
constexpr double field_tolerance = 1e-9;
constexpr double turn_tolerance = 1e-12;

std::string gear_label(const std::vector<int> & gears)
{
  std::string label;
  label.reserve(gears.size());
  for (int gear : gears) {
    label.push_back(gear > 0 ? '+' : '-');
  }
  return label;
}

std::vector<std::vector<int>> gear_combinations(std::size_t count, bool reverse)
{
  if (!reverse) {
    return {std::vector<int>(count, 1)};
  }
  if (count > 16) {
    throw std::invalid_argument("native successor search supports at most sixteen wheel signatures");
  }
  const auto combinations = std::size_t{1} << count;
  std::vector<std::vector<int>> result(combinations, std::vector<int>(count, 1));
  for (std::size_t index = 0; index < combinations; ++index) {
    for (std::size_t body = 0; body < count; ++body) {
      if ((index >> (count - body - 1)) & 1U) {
        result[index][body] = -1;
      }
    }
  }
  return result;
}

long double round_nine_digits(double input)
{
  // Use extended precision when scaling binary64 to 9 decimal places, with
  // ties-to-even independent of the ambient floating point rounding mode.
  if (!std::isfinite(input)) {
    throw std::invalid_argument("candidate control must be finite");
  }
  const long double scaled = static_cast<long double>(input) * 1000000000.0L;
  long double whole;
  const long double remainder = std::modf(scaled, &whole);
  if (std::abs(remainder) > 0.5L ||
    (std::abs(remainder) == 0.5L && std::fmod(whole, 2.0L) != 0.0L))
  {
    whole += std::copysign(1.0L, remainder);
  }
  return whole;
}

std::int64_t heading_key(double heading)
{
  return static_cast<std::int64_t>(round_nine_digits(wrap_angle(heading)));
}

void add_oblique(
  const TranslationSeed & seed, const std::vector<double> & headings, bool allow_reverse,
  std::vector<Candidate> & candidates)
{
  if (!(std::isfinite(seed.displacement.x) && std::isfinite(seed.displacement.y))) {
    throw std::invalid_argument("oblique seed displacement must be finite");
  }
  if (seed.displacement.x == 0.0 && seed.displacement.y == 0.0) {
    return;
  }
  const double bearing = std::atan2(seed.displacement.y, seed.displacement.x);
  Candidate branch{Primitive::oblique, seed.label, seed.displacement, 0.0, {}, {}};
  branch.bearing_rad = bearing;
  for (std::size_t i = 0; i < headings.size(); ++i) {
    double phi = wrap_angle(bearing - headings[i]);
    int direction = 1;
    if (std::abs(phi) > pi / 2.0 + turn_tolerance) {
      phi = wrap_angle(bearing - headings[i] + pi);
      direction = -1;
      if (!allow_reverse || std::abs(phi) > pi / 2.0 + turn_tolerance) {
        branch.refusal = Refusal::reverse_required;
        branch.failed_vehicle = i;
        candidates.push_back(std::move(branch));
        return;
      }
    }
    branch.directions.push_back(direction);
    branch.angles.push_back(std::clamp(phi, -pi / 2.0, pi / 2.0));
  }
  branch.label += "_sig" + gear_label(branch.directions);
  candidates.push_back(std::move(branch));
}

using SpinTargets = std::map<std::vector<std::int64_t>, Candidate>;

void add_ackermann(
  Primitive kind, const std::string & label, Point2d control, double delta,
  const std::vector<double> & base_tangents, const std::vector<double> & headings,
  const std::vector<std::vector<int>> & gears, bool valid_arc, bool prepare_spins,
  SpinTargets & targets, std::vector<Candidate> & candidates)
{
  for (const auto & gear : gears) {
    Candidate candidate{kind, label + "_gear" + gear_label(gear), control, delta, gear, {}};
    if (!valid_arc) {
      candidate.refusal = Refusal::arc_domain;
      candidates.push_back(std::move(candidate));
      continue;
    }
    std::vector<double> required;
    std::vector<std::int64_t> key;
    required.reserve(headings.size());
    key.reserve(headings.size());
    for (std::size_t i = 0; i < headings.size(); ++i) {
      const double target = wrap_angle(gear[i] > 0 ? base_tangents[i] : base_tangents[i] + pi);
      if (std::abs(wrap_angle(headings[i] - target)) > heading_tolerance) {
        candidate.refusal = Refusal::alignment_required;
      }
      required.push_back(target);
      if (prepare_spins) {
        key.push_back(heading_key(target));
      }
    }
    if (prepare_spins && targets.find(key) == targets.end()) {
      auto increments = spin_increments(headings, required);
      // An already aligned first representative still suppresses later targets
      // with this same decimal key. It simply emits no stationary self-loop.
      targets.emplace(std::move(key), Candidate{
        Primitive::spin, "S_to_" + candidate.label, {}, 0.0, {},
        increments ? std::move(*increments) : std::vector<double>{}});
    }
    candidates.push_back(std::move(candidate));
  }
}

double mode_switch_duration(double delta, SearchLimits limits)
{
  if (limits.fixed_setup) {
    return limits.mode_switch_s;
  }
  double steering = 0.0;
  if (delta > turn_tolerance) {
    const double rate = limits.steer_rate_radps;
    const double acceleration = limits.steer_accel_radps2;
    const double trapezoid = delta <= rate * rate / acceleration ?
      2.0 * std::sqrt(delta / acceleration) : delta / rate + rate / acceleration;
    const double quintic = std::max(1.875 * delta / rate,
        std::sqrt((10.0 / std::sqrt(3.0)) * delta / acceleration));
    steering = std::max(trapezoid, quintic);
  }
  return std::max(limits.mode_switch_s, limits.mode_setup_s + steering);
}

std::optional<std::vector<double>> arc_tangents(
  double yaw, const std::vector<Point2d> & offsets, const ArcSeed & seed, double minimum_radius)
{
  if (!(std::isfinite(seed.delta_rad) && std::isfinite(seed.pivot.x) && std::isfinite(seed.pivot.y))) {
    throw std::invalid_argument("arc seed pivot and turn must be finite");
  }
  if (seed.delta_rad == 0.0) {
    return std::nullopt;
  }
  std::vector<double> tangents;
  tangents.reserve(offsets.size());
  for (const auto & offset : offsets) {
    const double rx = offset.x - seed.pivot.x;
    const double ry = offset.y - seed.pivot.y;
    const double radius = static_cast<double>(std::hypot(
        static_cast<long double>(rx), static_cast<long double>(ry)));
    if (radius <= pivot_tolerance || radius < minimum_radius - field_tolerance) {
      return std::nullopt;
    }
    const double tangent = yaw + std::atan2(rx, -ry);
    tangents.push_back(seed.delta_rad > 0.0 ? tangent : tangent + pi);
  }
  return tangents;
}
}  // namespace

std::optional<std::vector<double>> spin_increments(
  const std::vector<double> & start, const std::vector<double> & target)
{
  if (start.size() != target.size()) {
    throw std::invalid_argument("spin requires one target per body");
  }
  std::vector<double> increments;
  increments.reserve(start.size());
  bool changed = false;
  for (std::size_t i = 0; i < start.size(); ++i) {
    const double delta = wrap_angle(target[i] - start[i]);
    if (!std::isfinite(delta)) {
      throw std::invalid_argument("spin headings must be finite");
    }
    changed = changed || std::abs(delta) > turn_tolerance;
    increments.push_back(delta);
  }
  if (!changed) {
    return std::nullopt;
  }
  return increments;
}

std::vector<Candidate> generate_candidates(
  double formation_yaw_rad, const std::vector<Point2d> & offsets,
  const std::vector<double> & body_headings, const std::vector<TranslationSeed> & oblique,
  const std::vector<TranslationSeed> & straight, const std::vector<ArcSeed> & arcs,
  double minimum_radius_m, bool allow_reverse, bool prepare_spins)
{
  const auto finite_point = [](Point2d p) {return std::isfinite(p.x) && std::isfinite(p.y);};
  if (offsets.empty() || offsets.size() != body_headings.size() ||
    !(std::isfinite(formation_yaw_rad) && std::isfinite(minimum_radius_m) && minimum_radius_m > 0.0) ||
    !std::all_of(offsets.begin(), offsets.end(), finite_point) ||
    !std::all_of(body_headings.begin(), body_headings.end(), [](double h) {return std::isfinite(h);}))
  {
    throw std::invalid_argument("invalid formation headings, offsets or minimum radius");
  }
  const auto gears = gear_combinations(offsets.size(), allow_reverse);
  std::vector<Candidate> candidates;
  SpinTargets targets;
  if (prepare_spins) {
    for (const auto & seed : oblique) {
      add_oblique(seed, body_headings, allow_reverse, candidates);
    }
  }
  for (const auto & seed : straight) {
    if (!(std::isfinite(seed.displacement.x) && std::isfinite(seed.displacement.y))) {
      throw std::invalid_argument("straight seed displacement must be finite");
    }
    if (seed.displacement.x == 0.0 && seed.displacement.y == 0.0) {
      continue;
    }
    const double bearing = std::atan2(seed.displacement.y, seed.displacement.x);
    add_ackermann(Primitive::straight, seed.label, seed.displacement, 0.0,
      std::vector<double>(offsets.size(), bearing), body_headings, gears, true,
      prepare_spins, targets, candidates);
  }
  for (const auto & seed : arcs) {
    auto tangents = arc_tangents(formation_yaw_rad, offsets, seed, minimum_radius_m);
    const bool valid = tangents.has_value();
    add_ackermann(Primitive::arc, seed.label, seed.pivot, seed.delta_rad,
      std::move(tangents).value_or(std::vector<double>{}), body_headings, gears, valid,
      prepare_spins, targets, candidates);
  }
  for (auto & item : targets) {
    if (!item.second.angles.empty()) {
      candidates.push_back(std::move(item.second));
    }
  }
  if (!prepare_spins) {
    // Direct translation checks A first, then O.
    for (const auto & seed : oblique) {
      add_oblique(seed, body_headings, allow_reverse, candidates);
    }
  }
  return candidates;
}

std::vector<Candidate> generate_lattice(
  double formation_yaw_rad, const std::vector<Point2d> & offsets,
  const std::vector<double> & headings, double step_m, std::optional<Point2d> target_step,
  const std::vector<Point2d> & pivots, double arc_step_rad, std::optional<double> target_yaw_rad,
  double minimum_radius_m, bool allow_reverse)
{
  std::vector<TranslationSeed> oblique;
  std::vector<TranslationSeed> straight;
  std::set<std::pair<long double, long double>> displacements;
  const double cosine = std::cos(formation_yaw_rad);
  const double sine = std::sin(formation_yaw_rad);
  for (std::size_t index = 0; index < 8; ++index) {
    const double angle = index * pi / 4.0;
    const double cargo_dx = std::cos(angle);
    const double cargo_dy = std::sin(angle);
    const Point2d delta{step_m * (cosine * cargo_dx - sine * cargo_dy),
      step_m * (sine * cargo_dx + cosine * cargo_dy)};
    if (displacements.emplace(round_nine_digits(delta.x), round_nine_digits(delta.y)).second) {
      oblique.push_back({"O_cargo_0" + std::to_string(index), delta});
    }
  }
  if (target_step) {
    straight.push_back({"A_straight_goal", *target_step});
    if (displacements.emplace(round_nine_digits(target_step->x),
        round_nine_digits(target_step->y)).second)
    {
      oblique.push_back({"O_goal", *target_step});
    }
  }

  std::vector<ArcSeed> arcs;
  std::set<std::tuple<double, double, long double>> rotations;
  const auto add_arc = [&](Point2d pivot, double delta, const char * prefix) {
      if (!rotations.emplace(pivot.x, pivot.y, round_nine_digits(delta)).second) {
        return;
      }
      std::ostringstream label;
      label.imbue(std::locale::classic());
      label << prefix << std::setprecision(6) << pivot.x << '_' << pivot.y << '_'
            << (delta > 0.0 ? "pos" : "neg");
      arcs.push_back({label.str(), pivot, delta});
    };
  for (const auto pivot : pivots) {
    add_arc(pivot, arc_step_rad, "A_arc_p");
    add_arc(pivot, -arc_step_rad, "A_arc_p");
  }
  if (target_yaw_rad) {
    const double cropped = wrap_angle(*target_yaw_rad - formation_yaw_rad);
    if (std::abs(cropped) > heading_tolerance) {
      for (const auto pivot : pivots) {
        if (std::abs(std::abs(cropped) - pi) <= heading_tolerance) {
          add_arc(pivot, pi, "A_arc_crop_p");
          add_arc(pivot, -pi, "A_arc_crop_p");
        } else {
          add_arc(pivot, cropped, "A_arc_crop_p");
        }
      }
    }
  }
  return generate_candidates(formation_yaw_rad, offsets, headings, oblique, straight, arcs,
    minimum_radius_m, allow_reverse, true);
}

std::optional<std::vector<GoalArcBranch>> goal_arc_branches(
  double formation_yaw_rad, const std::vector<Point2d> & offsets,
  const std::vector<double> & headings, Point2d pivot, double delta_rad,
  double minimum_radius_m, bool allow_reverse)
{
  if (offsets.empty() || offsets.size() != headings.size()) {
    throw std::invalid_argument("goal arc expects paired vehicle offsets and headings");
  }
  const auto tangents = arc_tangents(formation_yaw_rad, offsets, {{}, pivot, delta_rad}, minimum_radius_m);
  if (!tangents) {
    return std::nullopt;
  }
  const auto gears = gear_combinations(offsets.size(), allow_reverse);
  std::vector<GoalArcBranch> result;
  result.reserve(gears.size());
  for (const auto & choice : gears) {
    GoalArcBranch branch{choice, {}, 0.0, 0.0};
    for (std::size_t i = 0; i < choice.size(); ++i) {
      const double target = wrap_angle((*tangents)[i]) + (choice[i] < 0 ? pi : 0.0);
      const double spin = std::abs(wrap_angle(target - headings[i]));
      if (!std::isfinite(spin)) {
        throw std::invalid_argument("goal arc headings must be finite");
      }
      branch.target_headings.push_back(target);
      branch.maximum_spin_rad = std::max(branch.maximum_spin_rad, spin);
      branch.total_spin_rad += spin;
    }
    result.push_back(std::move(branch));
  }
  std::stable_sort(result.begin(), result.end(), [](const auto & a, const auto & b) {
      return std::tie(a.maximum_spin_rad, a.total_spin_rad) <
             std::tie(b.maximum_spin_rad, b.total_spin_rad);
    });
  return result;
}

std::array<double, 2> edge_cost(
  const std::vector<WheelSetup> & before, const std::vector<WheelSetup> & after,
  const std::vector<double> & center_travel_m, const std::vector<double> & yaw_travel_rad,
  SearchLimits limits)
{
  if (before.empty() || before.size() != after.size() ||
    after.size() != center_travel_m.size() || after.size() != yaw_travel_rad.size())
  {
    throw std::invalid_argument("edge cost requires matching wheel and motion arrays");
  }
  if (!std::isfinite(limits.mode_setup_s) || limits.mode_setup_s < 0.0 ||
    !std::isfinite(limits.mode_switch_s) || limits.mode_switch_s < 0.0 ||
    !std::isfinite(limits.steer_rate_radps) || limits.steer_rate_radps <= 0.0 ||
    !std::isfinite(limits.steer_accel_radps2) || limits.steer_accel_radps2 <= 0.0 ||
    !std::isfinite(limits.speed_mps) || limits.speed_mps <= 0.0 ||
    !std::isfinite(limits.yaw_rate_radps) || limits.yaw_rate_radps <= 0.0)
  {
    throw std::invalid_argument("invalid search cost limits");
  }
  const auto nonnegative = [](double value) {return std::isfinite(value) && value >= 0.0;};
  if (!std::all_of(center_travel_m.begin(), center_travel_m.end(), nonnegative) ||
    !std::all_of(yaw_travel_rad.begin(), yaw_travel_rad.end(), nonnegative))
  {
    throw std::invalid_argument("edge travel must be finite and nonnegative");
  }
  bool needs_setup = false;
  double setup = 0.0;
  for (std::size_t i = 0; i < before.size(); ++i) {
    double delta = 0.0;
    for (std::size_t wheel = 0; wheel < before[i].angles.size(); ++wheel) {
      const double change = std::abs(wrap_angle(after[i].angles[wheel] - before[i].angles[wheel]));
      if (!std::isfinite(change)) {
        throw std::invalid_argument("wheel change must be finite");
      }
      delta = std::max(delta, change);
    }
    needs_setup = needs_setup || before[i].mode != after[i].mode || delta > heading_tolerance;
    setup = std::max(setup, mode_switch_duration(delta, limits));
  }
  const double center = *std::max_element(center_travel_m.begin(), center_travel_m.end());
  const double yaw = *std::max_element(yaw_travel_rad.begin(), yaw_travel_rad.end());
  return {needs_setup ? setup : 0.0, std::max(center / limits.speed_mps, yaw / limits.yaw_rate_radps)};
}
}  // namespace fwis_multi_planner_cpp
