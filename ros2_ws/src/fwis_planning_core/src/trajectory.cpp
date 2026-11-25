#include "fwis_planning_core/trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace fwis_planning_core
{
namespace
{
Progress basic(double tau)
{
  return {
    std::pow(tau, 3.0) * (10.0 - 15.0 * tau + 6.0 * tau * tau),
    30.0 * tau * tau * (1.0 - tau) * (1.0 - tau),
    60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau),
    60.0 - 360.0 * tau + 360.0 * tau * tau};
}
}  // namespace

Progress quintic_progress(double tau)
{
  if (!std::isfinite(tau)) {
    throw std::invalid_argument("unit time must be finite");
  }
  if (tau <= 0.0) {
    return {0.0, 0.0, 0.0, 60.0};
  }
  if (tau >= 1.0) {
    return {1.0, 0.0, 0.0, 60.0};
  }
  if (tau > 0.5) {
    const Progress complement = basic(1.0 - tau);
    return {1.0 - complement.position, complement.first, -complement.second, complement.third};
  }
  return basic(tau);
}

TimedPrimitive::TimedPrimitive(Pose2d start, MotionPrimitive motion, double duration_s)
: start_(start), motion_(motion), duration_s_(duration_s)
{
  if (!std::isfinite(duration_s_) || duration_s_ <= 0.0) {
    throw std::invalid_argument("primitive duration must be finite and positive");
  }
  static_cast<void>(advance(start_, motion_));
}

TrajectorySample TimedPrimitive::sample(double time_s) const
{
  if (!std::isfinite(time_s) || time_s < 0.0 || time_s > duration_s_) {
    throw std::invalid_argument("sample time must be in the primitive's local closed interval");
  }
  const Progress progress = quintic_progress(time_s / duration_s_);
  const Pose2d pose = advance(start_, {motion_.mode, motion_.amount * progress.position, motion_.control});
  const double rate = motion_.amount * progress.first / duration_s_;
  Twist2d twist{};
  switch (motion_.mode) {
    case Mode::ackermann:
      twist = {rate, 0.0, motion_.control * rate};
      break;
    case Mode::oblique:
      twist = {rate * std::cos(motion_.control), rate * std::sin(motion_.control), 0.0};
      break;
    case Mode::spin:
      twist = {0.0, 0.0, rate};
      break;
  }
  if (!std::isfinite(twist.vx) || !std::isfinite(twist.vy) || !std::isfinite(twist.wz)) {
    throw std::overflow_error("sample twist is not finite");
  }
  return {time_s, pose, twist};
}
}  // namespace fwis_planning_core
