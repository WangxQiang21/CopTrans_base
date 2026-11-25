#pragma once

#include "fwis_planning_core/geometry.hpp"

namespace fwis_planning_core
{
struct Progress
{
  double position;       // unit progress, [0, 1]
  double first;          // derivative with respect to the unit time fraction
  double second;
  double third;
};

// Endpoint-stable quintic. Finite arguments outside [0, 1] return endpoint
// values as in UniformTimedPlan. Derivatives are with respect to unit time.
Progress quintic_progress(double tau);

struct TrajectorySample
{
  double time_s;
  Pose2d pose;
  Twist2d twist;
};

// Based on one exact constant-control primitive. Times are local to this
// primitive in seconds. Sampling does not replace continuous certification.
class TimedPrimitive
{
public:
  TimedPrimitive(Pose2d start, MotionPrimitive motion, double duration_s);
  TrajectorySample sample(double time_s) const;

private:
  Pose2d start_;
  MotionPrimitive motion_;
  double duration_s_;
};
}  // namespace fwis_planning_core
