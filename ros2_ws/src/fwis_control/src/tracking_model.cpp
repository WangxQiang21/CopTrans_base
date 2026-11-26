#include "fwis_control/tracking_core.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace fwis_control
{
namespace
{

void require_positive(std::initializer_list<double> values)
{
  if (!std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value) && value > 0.;
      }))
  {
    throw std::invalid_argument("controller settings must be finite and positive");
  }
}

double clamp(double value, double limit) noexcept
{
  return std::clamp(value, -limit, limit);
}

}  // namespace

bool valid_mode(int mode) noexcept
{
  return mode >= static_cast<int>(Mode::Ackermann) && mode <= static_cast<int>(Mode::Spin);
}

double wrap(double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::array<double, 3> Pose::error_to(const Pose & reference) const noexcept
{
  const double dx = reference.x - x;
  const double dy = reference.y - y;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * dx + s * dy, -s * dx + c * dy, wrap(reference.yaw - yaw)};
}

double Twist::speed() const noexcept {return std::hypot(vx, vy);}
bool Twist::zero() const noexcept {return vx == 0. && vy == 0. && wz == 0.;}
bool Twist::finite() const noexcept
{
  return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz);
}

Twist Twist::interpolate(const Twist & other, double fraction) const noexcept
{
  return {
    vx + fraction * (other.vx - vx),
    vy + fraction * (other.vy - vy),
    wz + fraction * (other.wz - wz)};
}

Twist between(const Pose & first, const Pose & last, double dt) noexcept
{
  const double dyaw = wrap(last.yaw - first.yaw);
  const double half = dyaw * .5;
  const double dx = last.x - first.x;
  const double dy = last.y - first.y;
  const double c = std::cos(first.yaw + half);
  const double s = std::sin(first.yaw + half);
  const double scale = half != 0. ? std::sin(half) / half : 1.;
  return {(c * dx + s * dy) / (dt * scale), (-s * dx + c * dy) / (dt * scale), dyaw / dt};
}

Pose advance(const Pose & pose, const Twist & twist, double dt) noexcept
{
  const double half = twist.wz * dt * .5;
  const double scale = half != 0. ? std::sin(half) / half : 1.;
  const double c = std::cos(pose.yaw + half);
  const double s = std::sin(pose.yaw + half);
  return {
    pose.x + dt * scale * (c * twist.vx - s * twist.vy),
    pose.y + dt * scale * (s * twist.vx + c * twist.vy),
    pose.yaw + twist.wz * dt};
}

double Point::time() const noexcept {return static_cast<double>(time_ns) * 1e-9;}
double Limits::guard_radius() const noexcept
{
  return std::max(mechanical_radius, switch_radius) + radius_margin;
}

void Limits::validate() const
{
  require_positive({
    max_speed, max_yaw_rate, max_accel, max_yaw_accel, mechanical_radius,
    switch_radius, radius_margin, parallel_min_vy, high_steer_angle, wheelbase, track});
  if (!std::isfinite(high_steer_speed) || high_steer_speed < 0. || high_steer_speed > max_speed ||
    high_steer_angle >= std::acos(0.))
  {
    throw std::invalid_argument("invalid steering speed envelope");
  }
  if (parallel_min_vy >= max_speed / 2.) {
    throw std::invalid_argument("parallel_min_vy must be well below max_speed");
  }
}

double Timing::period() const noexcept {return 1. / frequency;}
double Timing::minimum_hold() const noexcept
{
  return stable_time + switch_timeout + settle_time + 2. * period();
}

void Timing::validate() const
{
  require_positive({
    frequency, state_timeout, mode_timeout, plan_admission_timeout, clock_tolerance,
    approach_time, stable_time, switch_timeout, settle_time, stop_speed, stop_yaw_rate,
    final_stop_timeout});
}

void PreflightSettings::validate() const
{
  require_positive({
    min_step, max_step, stationary_position, stationary_yaw, stop_declared_speed,
    velocity_tolerance, oblique_yaw_tolerance, spin_position_tolerance, declared_speed_warn,
    declared_speed_tolerance, acceleration_tolerance});
  if (max_step <= min_step || declared_speed_tolerance <= declared_speed_warn) {
    throw std::invalid_argument("invalid preflight intervals or speed tolerances");
  }
}

void NonlinearGains::validate() const
{
  require_positive({ack_x, ack_y, ack_yaw, oblique_xy, spin_yaw});
}

void PIDGains::validate() const
{
  for (const double value : {kp, ki, kd, integral_limit, derivative_tau}) {
    if (!std::isfinite(value) || value < 0.) {
      throw std::invalid_argument("PID gains must be finite and nonnegative");
    }
  }
  if (derivative_tau == 0.) {
    throw std::invalid_argument("PID derivative time constant must be positive");
  }
}

void Configuration::validate() const
{
  limits.validate();
  timing.validate();
  preflight.validate();
  nonlinear.validate();
  for (const auto & gains : {ack_x, ack_y, ack_yaw, oblique, spin}) {
    gains.validate();
  }
}

void Reasons::add(std::string_view code) noexcept
{
  if (count < codes.size()) {
    codes[count++] = code;
  }
}

void SafetyFilter::reset() noexcept {previous_ = {};}

SafetyFilter::Result SafetyFilter::apply(
  Twist raw, Mode mode, double dt, double direction,
  std::optional<double> oblique_reference_vy, bool continuous_oblique) noexcept
{
  Reasons reasons;
  if (!raw.finite() || !std::isfinite(dt) || dt <= 0. ||
    !std::isfinite(direction) || (oblique_reference_vy && !std::isfinite(*oblique_reference_vy)) ||
    !valid_mode(static_cast<int>(mode)))
  {
    reset();
    reasons.add("nonfinite_or_invalid_interval");
    return {{}, reasons};
  }
  if (!previous_.zero() && mode_ != mode) {
    reasons.add("mode_change_requires_zero");
    raw = {};
    mode = mode_;
  }
  auto [vx, vy, wz] = raw;
  if (mode == Mode::Ackermann) {
    if (vy != 0.) {
      reasons.add("ackermann_no_lateral");
    }
    vy = 0.;
    if (direction != 0. && vx * direction < 0.) {
      vx = 0.;
      reasons.add("ackermann_direction");
    }
  } else if (mode == Mode::Oblique) {
    if (wz != 0.) {
      reasons.add("oblique_no_yaw");
    }
    wz = 0.;
  } else {
    if (vx != 0. || vy != 0.) {
      reasons.add("spin_no_translation");
    }
    vx = 0.;
    vy = 0.;
  }

  const double speed = std::hypot(vx, vy);
  if (speed > limits_.max_speed) {
    const double factor = limits_.max_speed / speed;
    vx *= factor;
    vy *= factor;
    reasons.add("speed");
  }
  if (std::abs(wz) > limits_.max_yaw_rate) {
    wz = clamp(wz, limits_.max_yaw_rate);
    reasons.add("yaw_rate");
  }

  if (mode == Mode::Ackermann) {
    const double safe_wz = clamp(wz, std::abs(vx) / limits_.guard_radius());
    if (safe_wz != wz) {
      reasons.add("ackermann_radius");
    }
    wz = safe_wz;
    if (previous_.vx * vx < 0.) {
      vx = 0.;
      wz = 0.;
      reasons.add("ackermann_reversal_through_zero");
    }
  } else if (mode == Mode::Oblique) {
    const double last_branch = previous_.vy != 0. ? previous_.vy : 1.;
    const double branch = oblique_reference_vy ?
      (std::abs(*oblique_reference_vy) > 1e-9 ? *oblique_reference_vy : last_branch) :
      (vy != 0. ? vy : last_branch);
    // A feedback correction may change the command within the planned half-plane.
    // Crossing its boundary would brake this vehicle independently of the formation.
    const bool crosses_reference = vy * branch < 0.;
    if (crosses_reference || (vx != 0. && std::abs(vy) < limits_.parallel_min_vy)) {
      vy = std::copysign(limits_.parallel_min_vy, branch);
      vx = clamp(vx, std::sqrt(limits_.max_speed * limits_.max_speed - vy * vy));
      reasons.add(crosses_reference ? "oblique_reference_branch" : "oblique_encoding");
    }
    // On a certified curve, a steering angle may pass through zero while
    // longitudinal travel continues. A true reversal still brakes to rest.
    const bool forward_progress = previous_.vx * vx > 0. &&
      previous_.vx * vx + previous_.vy * vy > 0.;
    if (previous_.vy * vy < 0. && !(continuous_oblique && forward_progress)) {
      vx = 0.;
      vy = 0.;
      reasons.add("oblique_reversal_through_zero");
    }
  }

  const double steering_cap = limits_.high_steer_speed;
  if (steering_cap > 0. && mode != Mode::Spin) {
    const double tangent = std::tan(limits_.high_steer_angle);
    const double ackermann_bound = 2. * tangent / (limits_.wheelbase + limits_.track * tangent);
    const auto high_steer = [&](Twist twist) {
        return mode == Mode::Ackermann ? std::abs(twist.wz) > ackermann_bound * std::abs(twist.vx) :
          std::abs(twist.vy) > tangent * std::abs(twist.vx);
      };
    const double current_speed = std::hypot(previous_.vx, previous_.vy);
    const double target_speed = std::hypot(vx, vy);
    if (current_speed > steering_cap && high_steer({vx, vy, wz})) {
      // Brake along the current direction before increasing steering.
      const double factor = std::min(steering_cap, target_speed) / current_speed;
      vx = previous_.vx * factor;
      vy = previous_.vy * factor;
      wz = previous_.wz * factor;
      reasons.add("high_steer_speed");
    } else if (target_speed > steering_cap &&
      (high_steer(previous_) || high_steer({vx, vy, wz})))
    {
      const double factor = steering_cap / target_speed;
      vx *= factor;
      vy *= factor;
      wz *= factor;
      reasons.add("high_steer_speed");
    }
  }
  const double linear_delta = std::hypot(vx - previous_.vx, vy - previous_.vy);
  const double angular_delta = std::abs(wz - previous_.wz);
  const double linear_fraction =
    linear_delta != 0. ? limits_.max_accel * dt / linear_delta : 1.;
  const double angular_fraction =
    angular_delta != 0. ? limits_.max_yaw_accel * dt / angular_delta : 1.;
  if (linear_fraction < 1.) {
    reasons.add("acceleration");
  }
  if (angular_fraction < 1.) {
    reasons.add("angular_acceleration");
  }
  const double step = std::min({1., linear_fraction, angular_fraction});
  const Twist desired{vx, vy, wz};
  Twist next = step == 1. ? desired : previous_.interpolate(desired, step);
  if (continuous_oblique && mode == Mode::Oblique && previous_.vy * vy < 0. &&
    next.vx != 0. && std::abs(next.vy) < limits_.parallel_min_vy)
  {
    // An axial Twist would passively select Ackermann. Choose the new side
    // only if its encoded speed is reachable in this actual control period.
    const double encoding = std::min({limits_.parallel_min_vy, std::abs(previous_.vy), std::abs(vy)});
    const double target_y = std::copysign(encoding, vy);
    next.vy = std::hypot(next.vx - previous_.vx, target_y - previous_.vy) <= limits_.max_accel * dt ?
      target_y : std::copysign(encoding, previous_.vy);
  }
  previous_ = next;
  mode_ = mode;
  return {previous_, reasons};
}

Feedback::Feedback(const Configuration & cfg, bool pid)
: nonlinear_(cfg.nonlinear), pid_(pid),
  axes_{{Axis{cfg.ack_x, false, 0., 0., 0., {}},
    Axis{cfg.ack_y, false, 0., 0., 0., {}}, Axis{cfg.ack_yaw, true, 0., 0., 0., {}},
    Axis{cfg.oblique, false, 0., 0., 0., {}}, Axis{cfg.oblique, false, 0., 0., 0., {}},
    Axis{cfg.spin, true, 0., 0., 0., {}}}}
{}

void Feedback::Axis::reset() noexcept
{
  integral = 0.;
  previous_integral = 0.;
  derivative = 0.;
  previous_error.reset();
}

void Feedback::reset() noexcept
{
  for (auto & axis : axes_) {
    axis.reset();
  }
  last_mode_.reset();
  raw_ = {};
}

double Feedback::Axis::update(double error, double dt) noexcept
{
  if (previous_error) {
    const double change = error - *previous_error;
    const double next = (angular ? wrap(change) : change) / dt;
    const double fraction = -std::expm1(-dt / gains.derivative_tau);
    derivative += fraction * (next - derivative);
  }
  previous_error = error;
  previous_integral = integral;
  integral = clamp(integral + gains.ki * error * dt, gains.integral_limit);
  return gains.kp * error + integral + gains.kd * derivative;
}

void Feedback::Axis::accept(double saturation) noexcept
{
  if ((integral - previous_integral) * saturation > 1e-12) {
    integral = previous_integral;
  }
}

Twist Feedback::update(
  const Reference & reference, const Observation & state, double dt, double direction) noexcept
{
  const auto [ex, ey, et] = state.pose.error_to(reference.pose);
  const auto [vx, vy, wz] = reference.twist;
  const double c = std::cos(et);
  const double s = std::sin(et);
  const Mode mode = reference.mode;

  if (pid_) {
    const double mode_direction = mode == Mode::Ackermann ? direction : 0.;
    if (last_mode_ != mode || last_direction_ != mode_direction) {
      reset();
      last_mode_ = mode;
      last_direction_ = mode_direction;
    }
    if (mode == Mode::Ackermann) {
      const double longitudinal = vx * c + axes_[0].update(ex, dt);
      double yaw = wz;
      yaw += axes_[1].update(direction * ey, dt);
      yaw += axes_[2].update(et, dt);
      raw_ = {longitudinal, 0., yaw};
    } else if (mode == Mode::Oblique) {
      raw_ = {c * vx - s * vy + axes_[3].update(ex, dt),
        s * vx + c * vy + axes_[4].update(ey, dt), 0.};
    } else {
      raw_ = {0., 0., wz + axes_[5].update(et, dt)};
    }
    return raw_;
  }

  if (mode == Mode::Ackermann) {
    return {vx * c + nonlinear_.ack_x * ex, 0.,
      wz + nonlinear_.ack_y * vx * ey + nonlinear_.ack_yaw * std::abs(vx) * s};
  }
  if (mode == Mode::Oblique) {
    return {c * vx - s * vy + nonlinear_.oblique_xy * ex,
      s * vx + c * vy + nonlinear_.oblique_xy * ey, 0.};
  }
  return {0., 0., wz + nonlinear_.spin_yaw * et};
}

void Feedback::accept(const Twist & applied) noexcept
{
  if (!pid_ || !last_mode_) {
    return;
  }
  if (*last_mode_ == Mode::Ackermann) {
    axes_[0].accept(raw_.vx - applied.vx);
    axes_[1].accept(raw_.wz - applied.wz);
    axes_[2].accept(raw_.wz - applied.wz);
  } else if (*last_mode_ == Mode::Oblique) {
    axes_[3].accept(raw_.vx - applied.vx);
    axes_[4].accept(raw_.vy - applied.vy);
  } else {
    axes_[5].accept(raw_.wz - applied.wz);
  }
}

}  // namespace fwis_control
