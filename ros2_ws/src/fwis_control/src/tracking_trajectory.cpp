#include "fwis_control/tracking_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fwis_control
{
namespace
{

constexpr double pi = 3.14159265358979323846;

bool finite(const Pose & pose) noexcept
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

}  // namespace

double Interval::middle() const noexcept {return (start + end) * .5;}

std::string_view Preflight::result() const noexcept
{
  if (issues.empty()) {
    return "PASS";
  }
  return valid ? "WARN" : "FAIL";
}

Preflight preflight(
  const Trajectory & trajectory, const Limits & limits, const PreflightSettings & settings,
  double switch_budget, Mode initial_mode, bool allow_axial_oblique)
{
  Preflight checked;
  auto issue = [&checked](
    std::string_view code, double start = 0., double end = 0., double value = 0.,
    double limit = 0., bool warning = false)
    {
      checked.issues.push_back({code, start, end, value, limit, warning});
    };
  const auto & points = trajectory.points;
  if (trajectory.plan_id == 0 || trajectory.stamp_ns <= 0 || trajectory.frame != "map") {
    issue("invalid_header");
  }
  if (points.size() < 2) {
    issue("too_few_points");
  }
  for (const Point & point : points) {
    if (point.time_ns < 0 || !valid_mode(point.mode) || !finite(point.pose) ||
      !std::isfinite(point.speed))
    {
      issue("invalid_point");
      break;
    }
  }
  if (!checked.issues.empty()) {
    return checked;
  }
  if (points.front().time_ns != 0) {
    issue("nonzero_start_time", points.front().time());
  }
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & a = points[i - 1];
    const auto & b = points[i];
    const double dt = static_cast<double>(b.time_ns - a.time_ns) * 1e-9;
    if (dt < settings.min_step || dt > settings.max_step) {
      issue("sample_interval", a.time(), b.time(), dt,
        dt > settings.max_step ? settings.max_step : settings.min_step);
    }
  }
  if (!checked.issues.empty()) {
    return checked;
  }

  auto check_rate = [&](const Twist & a, const Twist & b, double ta, double tb)
    {
      const double linear = std::hypot(b.vx - a.vx, b.vy - a.vy) / (tb - ta);
      const double angular = std::abs(b.wz - a.wz) / (tb - ta);
      if (linear > limits.max_accel + settings.acceleration_tolerance) {
        issue("linear_acceleration", ta, tb, linear, limits.max_accel);
      }
      if (angular > limits.max_yaw_accel + settings.acceleration_tolerance) {
        issue("angular_acceleration", ta, tb, angular, limits.max_yaw_accel);
      }
    };
  for (const Point * endpoint : {&points.front(), &points.back()}) {
    if (std::abs(endpoint->speed) > settings.stop_declared_speed) {
      issue("endpoint_speed", endpoint->time(), endpoint->time(), std::abs(endpoint->speed),
        settings.stop_declared_speed);
    }
  }
  const double tol = settings.velocity_tolerance;
  Pose mode_origin = points.front().pose;
  checked.intervals.reserve(points.size() - 1);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & a = points[i - 1];
    const auto & b = points[i];
    const double start = a.time();
    const double end = b.time();
    const Mode mode = static_cast<Mode>(a.mode);
    const double chord = std::hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y);
    const double raw_yaw_delta = b.pose.yaw - a.pose.yaw;
    if (!std::isfinite(chord) || !std::isfinite(raw_yaw_delta)) {
      issue("pose_delta_overflow", start, end);
      return checked;
    }
    const double yaw_delta = std::abs(wrap(raw_yaw_delta));
    if (std::abs(raw_yaw_delta) > 2. * pi + settings.stationary_yaw) {
      issue("yaw_discontinuity", start, end, std::abs(raw_yaw_delta), 2. * pi);
    }
    const bool stationary =
      chord <= settings.stationary_position && yaw_delta <= settings.stationary_yaw;
    if (a.mode != b.mode && (!stationary ||
      std::max(std::abs(a.speed), std::abs(b.speed)) > settings.stop_declared_speed))
    {
      issue("moving_mode_boundary", start, end);
    }
    const Twist velocity = stationary ? Twist{} : between(a.pose, b.pose, end - start);
    const double linear = velocity.speed();
    if (!velocity.finite() || !std::isfinite(linear)) {
      issue("nonfinite_interval_velocity", start, end);
      return checked;
    }
    checked.intervals.push_back({start, end, mode, velocity, stationary});
    const auto [vx, vy, wz] = velocity;
    if (linear > limits.max_speed + tol) {
      issue("linear_speed", start, end, linear, limits.max_speed);
    }
    if (std::abs(wz) > limits.max_yaw_rate + tol) {
      issue("yaw_rate", start, end, std::abs(wz), limits.max_yaw_rate);
    }
    const double declared = a.speed * .5 + b.speed * .5;
    double discrepancy{};
    if (mode == Mode::Ackermann) {
      discrepancy = std::abs(vx - declared);
      if (std::abs(vy) > tol) {
        issue("ackermann_lateral_velocity", start, end, std::abs(vy), tol);
      }
      if (limits.guard_radius() * std::abs(wz) > std::abs(vx) * 1.001 + 1e-8) {
        issue("ackermann_radius", start, end, std::abs(vx) / std::abs(wz), limits.guard_radius());
      }
    } else if (mode == Mode::Oblique) {
      discrepancy = std::abs(linear - std::abs(declared));
      if (std::abs(vx) > tol && vx * declared < 0.) {
        issue("oblique_direction", start, end, vx, declared);
      }
      if (!allow_axial_oblique && !stationary && linear > tol &&
        std::abs(vy) < limits.parallel_min_vy)
      {
        issue("oblique_twist_encodes_ackermann", start, end, std::abs(vy), limits.parallel_min_vy);
      }
      const double drift = std::abs(wrap(b.pose.yaw - mode_origin.yaw));
      if (std::abs(wz) > tol || drift > settings.oblique_yaw_tolerance) {
        issue("oblique_yaw_change", start, end, drift, settings.oblique_yaw_tolerance);
      }
    } else {
      discrepancy = std::max(std::abs(a.speed), std::abs(b.speed));
      const double drift = std::hypot(b.pose.x - mode_origin.x, b.pose.y - mode_origin.y);
      if (linear > tol || drift > settings.spin_position_tolerance) {
        issue("spin_translation", start, end, drift, settings.spin_position_tolerance);
      }
    }
    if (discrepancy > settings.declared_speed_warn) {
      issue("declared_speed_mismatch", start, end, discrepancy,
        settings.declared_speed_tolerance, discrepancy <= settings.declared_speed_tolerance);
    }
    if (a.mode != b.mode) {
      mode_origin = b.pose;
    }
  }

  const auto & intervals = checked.intervals;
  for (std::size_t i = 0; i < intervals.size(); ++i) {
    const auto & current = intervals[i];
    if (current.stationary) {
      continue;
    }
    if (i == 0 || intervals[i - 1].stationary || intervals[i - 1].mode != current.mode) {
      check_rate({}, current.velocity, current.start, current.middle());
    } else {
      const auto & previous = intervals[i - 1];
      check_rate(previous.velocity, current.velocity, previous.middle(), current.middle());
    }
    if (i + 1 == intervals.size() || intervals[i + 1].stationary ||
      intervals[i + 1].mode != current.mode)
    {
      check_rate(current.velocity, {}, current.middle(), current.end);
    }
  }

  int old_mode = static_cast<int>(initial_mode);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & point = points[i];
    if (point.mode == old_mode) {
      continue;
    }
    const Mode target = static_cast<Mode>(point.mode);
    std::size_t tail = i;
    while (tail < intervals.size() && intervals[tail].mode == target &&
      intervals[tail].stationary && points[tail + 1].mode == point.mode &&
      std::max(std::abs(points[tail].speed), std::abs(points[tail + 1].speed)) <=
      settings.stop_declared_speed)
    {
      ++tail;
    }
    const double window = points[tail].time() - point.time();
    if (window + 1e-9 < switch_budget) {
      issue("insufficient_mode_hold", point.time(), points[tail].time(), window, switch_budget);
    }
    checked.switches.push_back({point.time(), points[tail].time(), target});
    old_mode = point.mode;
  }
  checked.valid = std::none_of(
    checked.issues.begin(), checked.issues.end(),
    [](const Issue & item) {return !item.warning;});
  return checked;
}

Reference sample(const Trajectory & trajectory, const Preflight & checked, double time) noexcept
{
  const auto & points = trajectory.points;
  const auto & intervals = checked.intervals;
  time = std::clamp(time, 0., points.back().time());
  const auto upper = std::upper_bound(points.begin(), points.end(), time,
    [](double value, const Point & point) {return value < point.time();});
  const std::size_t i = static_cast<std::size_t>(std::distance(points.begin(), upper) - 1);
  const auto & point = points[i];
  const auto mode = static_cast<Mode>(point.mode);
  if (i == intervals.size() || intervals[i].stationary) {
    return {time, point.pose, {}, mode};
  }
  const auto & current = intervals[i];
  const auto position = advance(point.pose, current.velocity, time - point.time());
  const bool before_middle = time < current.middle();
  double anchor_time = before_middle ? current.start : current.end;
  Twist anchor;
  if ((before_middle && i > 0) || (!before_middle && i + 1 < intervals.size())) {
    const auto & adjacent = intervals[before_middle ? i - 1 : i + 1];
    if (adjacent.mode == current.mode && !adjacent.stationary) {
      anchor_time = adjacent.middle();
      anchor = adjacent.velocity;
    }
  }
  const double fraction = (time - current.middle()) / (anchor_time - current.middle());
  return {time, position, current.velocity.interpolate(anchor, fraction), mode};
}

const Preflight & PreparedPlan::checked(Mode initial_mode) const noexcept
{
  return trajectory.points.empty() || trajectory.points.front().mode == static_cast<int>(initial_mode) ?
    matching_mode : different_mode;
}

PreparedPlan prepare(Trajectory trajectory, const Configuration & config, bool allow_axial_oblique)
{
  const bool has_points = !trajectory.points.empty();
  const int first_mode = has_points ? trajectory.points.front().mode : 0;
  const Mode matching = valid_mode(first_mode) ? static_cast<Mode>(first_mode) : Mode::Ackermann;
  const Mode different = matching == Mode::Ackermann ? Mode::Oblique : Mode::Ackermann;
  PreparedPlan plan{std::move(trajectory), {}, {}};
  plan.matching_mode = preflight(
    plan.trajectory, config.limits, config.preflight, config.timing.minimum_hold(),
    matching, allow_axial_oblique);
  if (has_points) {
    plan.different_mode = preflight(
      plan.trajectory, config.limits, config.preflight, config.timing.minimum_hold(),
      different, allow_axial_oblique);
  }
  return plan;
}

}  // namespace fwis_control
