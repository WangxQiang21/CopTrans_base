#include "fwis_planning_core/geometry.hpp"

#include <cmath>
#include <stdexcept>

namespace fwis_planning_core
{
double wrap_angle(double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Transform2d::Transform2d(Pose2d pose) noexcept
: x_(pose.x), y_(pose.y), cos_yaw_(std::cos(pose.yaw)), sin_yaw_(std::sin(pose.yaw))
{
}

Point2d Transform2d::apply(Point2d local) const noexcept
{
  return {
    x_ + cos_yaw_ * local.x - sin_yaw_ * local.y,
    y_ + sin_yaw_ * local.x + cos_yaw_ * local.y};
}

Point2d formation_center(Pose2d reference, Point2d offset) noexcept
{
  return Transform2d(reference).apply(offset);
}

Pose2d reference_from_pivot(Point2d world_pivot, Point2d local_pivot, double yaw) noexcept
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {world_pivot.x - c * local_pivot.x + s * local_pivot.y,
    world_pivot.y - s * local_pivot.x - c * local_pivot.y, yaw};
}

Pose2d advance(Pose2d start, MotionPrimitive motion)
{
  if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.yaw) ||
    !std::isfinite(motion.amount) || !std::isfinite(motion.control))
  {
    throw std::invalid_argument("pose and primitive must be finite");
  }

  Pose2d result = start;
  switch (motion.mode) {
    case Mode::ackermann: {
      const double yaw = start.yaw + motion.amount * motion.control;
      if (std::abs(motion.control) < 1e-10) {
        result.x += motion.amount * std::cos(start.yaw);
        result.y += motion.amount * std::sin(start.yaw);
      } else {
        result.x += (std::sin(yaw) - std::sin(start.yaw)) / motion.control;
        result.y += (std::cos(start.yaw) - std::cos(yaw)) / motion.control;
      }
      result.yaw = wrap_angle(yaw);
      break;
    }
    case Mode::oblique: {
      const double direction = start.yaw + motion.control;
      result.x += motion.amount * std::cos(direction);
      result.y += motion.amount * std::sin(direction);
      break;
    }
    case Mode::spin:
      result.yaw = wrap_angle(start.yaw + motion.amount);
      break;
    default:
      throw std::invalid_argument("unsupported primitive mode");
  }

  if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.yaw)) {
    throw std::overflow_error("primitive result is not finite");
  }
  return result;
}
}  // namespace fwis_planning_core
