// Run the two shipped tracking algorithms on a native oblique model whose
// lateral velocity changes sign at mid-phase; sample through the production API.
#include "../src/hybrid_tracking.hpp"
#include <iomanip>
#include <iostream>

using namespace fwis_control;
namespace hj = fwis_hybrid_joint;

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const bool pid = std::string(argv[1]) == "pid";
  const double gear = std::stoi(argv[2]);
  Configuration config;
  auto model = std::make_shared<hybrid_ros::Decoded>();
  auto &request = model->context.request;
  request.vehicles.push_back({0., 0., .818, .58});
  request.limits.speed = .25;
  request.limits.yaw_rate = request.limits.spin_speed = .4;
  request.limits.yaw_acceleration = request.limits.spin_acceleration = .5;
  model->context.id = 1;
  hj::Phase motion;
  motion.mode = hj::Mode::O;
  motion.duration = 8.;
  motion.gear.fill(static_cast<int>(gear));
  auto &piece = motion.pieces.emplace_back();
  for (int i = 0; i < 6; ++i) piece.x[i] = gear * .8 * i / 5.;
  // Bernstein degree elevation of y(s) = (1/8) s(1-s).
  piece.y = {0., .025, .0375, .0375, .025, 0.};
  model->plan.phases = {motion};
  const auto plan = prepare_hybrid(model, 1, 1000000000, "crossing_fixture", config);
  Execution engine(config, pid);
  if (!engine.admit(plan, Mode::Oblique, 1.).valid) return 3;
  std::cout << std::setprecision(17);
  for (int k = 0; k <= 206; ++k) {
    const double now = 1. + k *.04;
    const auto ref = plan.model_sample(now-1.);
    auto pose = ref.pose;
    pose.x -= .012 * ref.twist.vx;
    pose.y += .008 * ref.twist.vy;
    Observation feedback{pose, ref.twist, now, Mode::Oblique, now};
    const auto output = engine.tick(now, feedback);
    if (!output.reference || output.requested_mode) return 4;
    std::cout << k << ' ' << name(output.phase) << ' ' << ref.twist.vx << ' ' << ref.twist.vy << ' '
              << output.raw.vx << ' ' << output.raw.vy << ' ' << output.raw.wz << ' '
              << output.command.vx << ' ' << output.command.vy << ' ' << output.command.wz << '\n';
    if (output.phase == Phase::Complete) return 0;
  }
  return 5;
}
