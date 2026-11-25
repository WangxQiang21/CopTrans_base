#include "fwis_control/tracking_core.hpp"

#include <algorithm>
#include <cmath>

namespace fwis_control
{
namespace
{

bool fresh(double age, double future_tolerance, double timeout) noexcept
{
  return age >= -future_tolerance && age <= timeout;
}

}  // namespace

std::string_view name(Phase phase) noexcept
{
  constexpr std::array<std::string_view, 13> names{
    "IDLE", "WAIT_START", "TRACKING", "APPROACH_SWITCH", "STOPPING", "REQUEST_MODE_SWITCH",
    "WAIT_MODE_CONFIRMATION", "SETTLE", "FINAL_STOP", "COMPLETE", "STOPPED", "REJECTED",
    "STALE_TRAJECTORY"};
  return names[static_cast<std::size_t>(phase)];
}

Execution::Execution(const Configuration & config, bool pid)
: timing_(config.timing), controller_(config, pid), filter_(config.limits)
{}

const Preflight & Execution::admit(
  const PreparedPlan & plan, Mode initial_mode, double now, double future_window) noexcept
{
  controller_.reset();
  filter_.reset();
  plan_ = nullptr;
  checked_ = &plan.checked(initial_mode);
  start_ = static_cast<double>(plan.trajectory.stamp_ns) * 1e-9;
  if (!checked_->valid) {
    phase_ = Phase::Rejected;
    return *checked_;
  }
  const double age = now - start_;
  if (!fresh(age, std::max(timing_.clock_tolerance, future_window),
    timing_.plan_admission_timeout) || age >= plan.trajectory.points.back().time())
  {
    phase_ = Phase::StaleTrajectory;
    return *checked_;
  }
  plan_ = &plan;
  switch_index_ = 0;
  expected_mode_ = initial_mode;
  last_tick_.reset();
  still_since_.reset();
  request_at_.reset();
  settle_from_.reset();
  direction_ = 0.;
  phase_ = Phase::Tracking;
  return *checked_;
}

Output Execution::abort(std::string_view reason, std::optional<Reference> reference) noexcept
{
  plan_ = nullptr;
  checked_ = nullptr;
  filter_.reset();
  controller_.reset();
  phase_ = Phase::Stopped;
  Reasons reasons;
  reasons.add(reason);
  return {phase_, {}, {}, reference, {}, reasons};
}

bool Execution::feedback_is_fresh(
  const std::optional<Observation> & obs, double now) const noexcept
{
  return obs && std::isfinite(obs->pose.x) && std::isfinite(obs->pose.y) &&
    std::isfinite(obs->pose.yaw) && obs->twist.finite() &&
    valid_mode(static_cast<int>(obs->mode)) &&
    fresh(now - obs->stamp, timing_.clock_tolerance, timing_.state_timeout) &&
    fresh(now - obs->mode_stamp, timing_.clock_tolerance, timing_.mode_timeout);
}

bool Execution::stopped(const Observation & obs) const noexcept
{
  return obs.twist.speed() <= timing_.stop_speed &&
    std::abs(obs.twist.wz) <= timing_.stop_yaw_rate && filter_.previous().zero();
}

Output Execution::brake(
  Phase phase, std::optional<Reference> reference, Mode mode, double dt,
  std::optional<Mode> requested) noexcept
{
  phase_ = phase;
  const auto result = filter_.apply({}, mode, dt);
  return {phase, result.command, {}, reference, requested, result.reasons};
}

Output Execution::tick(double now, const std::optional<Observation> & obs) noexcept
{
  if (!plan_) {
    return {phase_, {}, {}, {}, {}, {}};
  }
  if (!std::isfinite(now) || !feedback_is_fresh(obs, now)) {
    return abort("missing_stale_or_invalid_feedback");
  }
  const double dt = last_tick_ ? now - *last_tick_ : timing_.period();
  if (!(dt > 0. && dt <= timing_.state_timeout)) {
    return abort("control_clock_jump_or_overrun");
  }
  last_tick_ = now;
  const double elapsed = now - start_;
  if (elapsed < 0.) {
    return brake(Phase::WaitStart, {}, expected_mode_, dt);
  }
  Reference reference;
  try {
    reference = plan_->model_sample ? plan_->model_sample(elapsed) :
      sample(plan_->trajectory, *checked_, elapsed);
  } catch (...) {
    return abort("invalid_model_sample");
  }
  if (!std::isfinite(reference.pose.x) || !std::isfinite(reference.pose.y) ||
      !std::isfinite(reference.pose.yaw) || !reference.twist.finite()) {
    return abort("invalid_model_sample");
  }
  const Switch * event = switch_index_ < checked_->switches.size() ?
    &checked_->switches[switch_index_] : nullptr;
  if (event && elapsed + 1e-8 >= event->time) {
    if (const auto output = transition(*event, now, elapsed, *obs, reference, dt)) {
      return *output;
    }
    event = nullptr;
  }
  if (obs->mode != expected_mode_) {
    return abort("unexpected_feedback_mode", reference);
  }
  const double duration = plan_->trajectory.points.back().time();
  if (elapsed + 1e-8 >= duration) {
    if (stopped(*obs)) {
      phase_ = Phase::Complete;
      plan_ = nullptr;
      checked_ = nullptr;
      return {phase_, {}, {}, reference, {}, {}};
    }
    if (elapsed > duration + timing_.final_stop_timeout) {
      return abort("final_stop_timeout", reference);
    }
    return brake(Phase::FinalStop, reference, expected_mode_, dt);
  }
  if (reference.mode != expected_mode_) {
    return abort("unconfirmed_planned_mode", reference);
  }
  phase_ = event && elapsed >= event->time - timing_.approach_time ?
    Phase::ApproachSwitch : Phase::Tracking;
  if (reference.twist.zero()) {
    controller_.reset();
    return brake(phase_, reference, expected_mode_, dt);
  }
  if (reference.mode == Mode::Ackermann && std::abs(reference.twist.vx) > 1e-9) {
    direction_ = std::copysign(1., reference.twist.vx);
  }
  const auto raw = controller_.update(reference, *obs, dt, direction_);
  const auto result =
    filter_.apply(raw, reference.mode, dt, direction_, reference.twist.vy);
  if (result.reasons.count && result.reasons.codes[0] == "nonfinite_or_invalid_interval") {
    return abort("invalid_controller_output", reference);
  }
  controller_.accept(result.command);
  return {phase_, result.command, raw, reference, {}, result.reasons};
}

std::optional<Output> Execution::transition(
  const Switch & event, double now, double elapsed, const Observation & obs,
  const Reference & reference, double dt) noexcept
{
  if (settle_from_) {
    if (obs.mode != event.mode || !stopped(obs)) {
      return abort("mode_or_stop_lost_during_settle", reference);
    }
    if (now - *settle_from_ + 1e-8 >= timing_.settle_time) {
      ++switch_index_;
      expected_mode_ = event.mode;
      still_since_.reset();
      request_at_.reset();
      settle_from_.reset();
      direction_ = 0.;
      controller_.reset();
      return {};
    }
    if (elapsed + 1e-8 >= event.resume) {
      return abort("planned_hold_exhausted", reference);
    }
    return brake(Phase::Settle, reference, event.mode, dt);
  }
  if (elapsed + 1e-8 >= event.resume) {
    return abort("planned_hold_exhausted", reference);
  }
  if (request_at_) {
    if (obs.mode == event.mode && obs.mode_stamp + 1e-8 >= *request_at_) {
      settle_from_ = now;
      return brake(Phase::Settle, reference, event.mode, dt);
    }
    if (now - *request_at_ > timing_.switch_timeout) {
      return abort("mode_confirmation_timeout", reference);
    }
    if (obs.mode != expected_mode_) {
      return abort("unexpected_feedback_mode", reference);
    }
    return brake(Phase::WaitModeConfirmation, reference, expected_mode_, dt, event.mode);
  }
  if (obs.mode != expected_mode_) {
    return abort("unexpected_feedback_mode", reference);
  }
  if (!stopped(obs)) {
    still_since_.reset();
  } else if (!still_since_) {
    still_since_ = obs.stamp;
  } else if (obs.stamp - *still_since_ + 1e-8 >= timing_.stable_time) {
    request_at_ = now;
    return brake(Phase::RequestModeSwitch, reference, expected_mode_, dt, event.mode);
  }
  return brake(Phase::Stopping, reference, expected_mode_, dt);
}

}  // namespace fwis_control
