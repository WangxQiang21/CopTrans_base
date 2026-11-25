// Exercise the ordinary model-to-controller path on an optimized curved O,
// including its actual steering preparation and its terminal stop.
#include "../src/hybrid_tracking.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

namespace hj = fwis_hybrid_joint;
namespace ctrl = fwis_control;

int main()
{
  auto model = std::make_shared<hybrid_ros::Decoded>();
  model->context.id = 1;
  auto & request = model->context.request;
  std::vector<std::uint8_t> cells(60 * 60);
  for (int row = 28; row <= 31; ++row) {
    for (int col = 29; col <= 30; ++col) cells[row * 60 + col] = 1;
  }
  request.grid = std::make_shared<hj::GridSnapshot>(
    60, 60, .25, fwis_planning_core::Point2d{-7.5, -7.5}, cells);
  request.vehicles = {{1., 0., .838, .6}, {0., 1., .838, .6}, {0., -1., .838, .6}};
  request.cargo = hj::Footprint{.5, 0., 2.1, 2.8};
  request.start.reference = {-5., 0., 0.};
  request.goal.reference = {5., 0., 0.};
  request.limits.speed = .25;
  request.limits.yaw_rate = request.limits.spin_speed = .4;
  request.limits.yaw_acceleration = request.limits.spin_acceleration = .5;
  hj::SolveOptions options;
  options.initial_sequence = {hj::Mode::O};
  options.enable_edits = false;
  auto deadline = []{return hj::Clock::now() + std::chrono::seconds(8);};
  auto solved = hj::solve(request, deadline(), {}, options);
  if (!solved.plan || !hj::certify(request, *solved.plan, deadline()).executable) return 1;
  model->plan = std::move(*solved.plan);
  ctrl::Configuration config;
  for (unsigned id = 1; id <= request.vehicles.size(); ++id) {
    const auto plan = ctrl::prepare_hybrid(model, id, 1000000000, "solved_oblique", config);
    for (bool pid : {false, true}) {
      ctrl::Execution engine(config, pid);
      auto mode = ctrl::Mode::Ackermann;
      if (!engine.admit(plan, mode, 1.).valid) return 2;
      bool positive = false, negative = false, complete = false;
      int requests = 0;
      const double end = hj::duration(model->plan);
      for (int k = 0; k * .04 <= end + 2.; ++k) {
        const double t = k * .04, now = 1. + t;
        const auto ref = plan.model_sample(t);
        const ctrl::Observation feedback{ref.pose, ref.twist, now, mode, now};
        const auto out = engine.tick(now, feedback);
        if (!out.command.finite() || out.command.wz != 0.) return 3;
        if (out.requested_mode) {
          if (*out.requested_mode != ctrl::Mode::Oblique || !out.command.zero()) return 4;
          mode = *out.requested_mode;
          ++requests;
        }
        if (std::abs(ref.twist.vx) > .02 && std::abs(ref.twist.vy) > .005) {
          if (mode != ctrl::Mode::Oblique || out.command.zero()) return 5;
          positive |= out.command.vy > .003;
          negative |= out.command.vy < -.003;
        }
        if (out.phase == ctrl::Phase::Complete) {
          if (!out.command.zero() || !ref.twist.zero()) return 6;
          complete = true;
          break;
        }
      }
      if (!complete || !positive || !negative || requests == 0) return 7;
    }
  }
  std::cout << "Three solved native vehicle references completed both tracking algorithms\n";
  return 0;
}
