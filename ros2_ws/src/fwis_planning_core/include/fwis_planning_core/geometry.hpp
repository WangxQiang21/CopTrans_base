#pragma once

#include <cstdint>

namespace fwis_planning_core
{
struct Point2d
{
  double x;  // metres
  double y;
};

struct Pose2d
{
  double x;  // metres, in the fixed frame
  double y;
  double yaw;  // radians, counterclockwise from the fixed x axis
};

struct Twist2d
{
  double vx;  // body-frame m/s
  double vy;
  double wz;  // rad/s
};

enum class Mode : std::uint8_t
{
  ackermann = 0,
  oblique = 1,
  spin = 2,
};

struct MotionPrimitive
{
  Mode mode;
  double amount;  // signed distance in metres for A/O, signed radians for S
  double control;  // signed curvature [1/m] for A, body oblique angle [rad] for O, unused for S
};

// Equivalent to atan2(sin(angle), cos(angle)), including the sign on the pi boundary.
double wrap_angle(double angle) noexcept;

class Transform2d
{
public:
  // Captures an immutable local-to-fixed transform from a pose in the fixed frame.
  explicit Transform2d(Pose2d pose) noexcept;
  Point2d apply(Point2d local) const noexcept;

private:
  double x_, y_, cos_yaw_, sin_yaw_;
};

Point2d formation_center(Pose2d reference, Point2d offset) noexcept;

// Invert the offset placement for a fixed world pivot and supplied orientation.
Pose2d reference_from_pivot(Point2d world_pivot, Point2d local_pivot, double yaw) noexcept;

// Same constant-control integration as the Python vehicle-centre oracle.
// Oblique translation preserves the given yaw. A and S wrap their result.
// Non-finite inputs/results and unsupported modes are rejected.
Pose2d advance(Pose2d start, MotionPrimitive motion);
}  // namespace fwis_planning_core
