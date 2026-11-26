#pragma once
#include "fwis_control/tracking_core.hpp"
#include "fwis_hybrid_joint_ros/transport.hpp"
#include <algorithm>
#include <cmath>

namespace fwis_control
{
// Build all immutable state on the receiver thread. The group coordinator makes
// its own continuous proof against the independently received request/map.
inline PreparedPlan prepare_hybrid(
  std::shared_ptr<const hybrid_ros::Decoded> model, uint32_t vehicle_id, int64_t epoch,
  const std::string & hash, const Configuration & config)
{
  namespace h = fwis_hybrid_joint;
  const auto & r = model->context.request;
  const auto & ids = model->context.ids;
  const auto vehicle = std::find(ids.begin(), ids.end(), vehicle_id);
  if (vehicle == ids.end()) {
    throw std::invalid_argument("vehicle missing from hybrid plan");
  }
  const auto & l = r.limits;
  const auto & tracker = config.limits;
  const bool compatible_steering_speed = tracker.high_steer_speed <= 0. ||
    l.speed <= tracker.high_steer_speed ||
    (l.high_steer_speed > 0. && l.high_steer_speed <= tracker.high_steer_speed &&
    l.high_steer_angle <= tracker.high_steer_angle && l.wheelbase == tracker.wheelbase &&
    l.track == tracker.track);
  if (!(l.radius >= config.limits.guard_radius() && l.speed <= config.limits.max_speed &&
    l.spin_speed <= config.limits.max_yaw_rate && l.yaw_rate > 0 &&
    l.yaw_rate <= config.limits.max_yaw_rate &&
    l.acceleration <= config.limits.max_accel && l.spin_acceleration <= config.limits.max_yaw_accel &&
    l.yaw_acceleration > 0 && l.yaw_acceleration <= config.limits.max_yaw_accel &&
    compatible_steering_speed))
  {
    throw std::invalid_argument("model limits exceed controller envelope");
  }
  auto reference = [model, index = std::distance(ids.begin(), vehicle)](double t) {
      const auto s = h::evaluate(model->context.request, model->plan, t);
      const auto & b = s.bodies[index];
      return Reference{t, {b.pose.x, b.pose.y, b.pose.yaw}, {b.vx, b.vy, b.wz}, static_cast<Mode>(s.mode)};
    };
  PreparedPlan result;
  result.trajectory.stamp_ns = epoch;
  result.trajectory.frame = "map";
  result.trajectory.plan_id = model->context.id;
  result.model_sha256 = hash;
  result.model_sample = reference;
  for (double t:{0., h::duration(model->plan)}) {
    auto v = reference(t);
    if (!v.twist.finite() || !v.twist.zero()) {
      throw std::invalid_argument("moving model endpoint");
    }
    result.trajectory.points.push_back({std::llround(t * 1e9), v.pose, 0, int(v.mode)});
  }
  const auto first = Mode(model->plan.phases.front().mode);
  auto check = [&](Mode mode) {
      Preflight p;
      p.valid = true;
      double t = 0.;
      for (const auto & phase:model->plan.phases) {
        auto target = Mode(phase.mode);
        if (target != mode) {
          if (phase.kind == h::Kind::Motion || phase.duration + 1e-9 < config.timing.minimum_hold()) {
            p.issues.push_back({"insufficient_mode_hold", t, t + phase.duration, phase.duration,
                config.timing.minimum_hold(), false});
            p.valid = false;
          }
          p.switches.push_back({t, t + phase.duration, target});
          mode = target;
        }
        t += phase.duration;
      }
      return p;
    };
  result.matching_mode = check(first);
  result.different_mode = check(first == Mode::Ackermann ? Mode::Spin : Mode::Ackermann);
  return result;
}
}
