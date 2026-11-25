#include "fwis_control/tracking_core.hpp"

#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

int main(int argc, char ** argv)
{
  using namespace fwis_control;
  std::cin.tie(nullptr);
  std::ios::sync_with_stdio(false);
  const bool measure = argc > 1 && std::string(argv[1]) == "quiet";
  std::cout << std::setprecision(17);
  Configuration config;
  int pid, allow_axial, initial;
  double admitted_at, future_window;
  while (std::cin >> pid >> allow_axial >> initial >> admitted_at >> future_window) {
    Trajectory trajectory;
    std::size_t points;
    if (!(std::cin >> trajectory.stamp_ns >> trajectory.frame >> trajectory.plan_id >> points)) {
      return 1;
    }
    for (std::size_t i = 0; i < points; ++i) {
      Point point;
      if (!(std::cin >> point.time_ns >> point.pose.x >> point.pose.y >> point.pose.yaw >>
        point.speed >> point.mode))
      {
        return 1;
      }
      trajectory.points.push_back(point);
    }
    const auto plan = prepare(std::move(trajectory), config, allow_axial != 0);
    Execution engine(config, pid != 0);
    const auto & result =
      engine.admit(plan, static_cast<Mode>(initial), admitted_at, future_window);
    std::cout << result.result() << ' ' << name(engine.phase());
    for (const auto & issue : result.issues) {
      std::cout << ' ' << issue.code << ':' << (issue.warning ? "WARN" : "FAIL");
    }
    std::cout << '\n';
    std::size_t ticks;
    if (!(std::cin >> ticks)) {
      return 1;
    }
    for (std::size_t i = 0; i < ticks; ++i) {
      double now;
      int has_feedback, mode, inject_nan;
      Observation obs;
      if (!(std::cin >> now >> has_feedback >> obs.pose.x >> obs.pose.y >> obs.pose.yaw >>
        obs.twist.vx >> obs.twist.vy >> obs.twist.wz >> obs.stamp >> mode >>
        obs.mode_stamp >> inject_nan))
      {
        return 1;
      }
      obs.mode = static_cast<Mode>(mode);
      if (inject_nan) {
        obs.pose.x = std::numeric_limits<double>::quiet_NaN();
      }
      const auto out = engine.tick(now, has_feedback ? std::optional<Observation>(obs) : std::nullopt);
      if (measure) {
        continue;
      }
      std::cout << name(out.phase) << ' '
                << (out.requested_mode ? static_cast<int>(*out.requested_mode) : -1) << ' '
                << out.raw.vx << ' ' << out.raw.vy << ' ' << out.raw.wz << ' '
                << out.command.vx << ' ' << out.command.vy << ' ' << out.command.wz << ' ';
      if (out.reference) {
        const auto & ref = *out.reference;
        std::cout << static_cast<int>(ref.mode) << ' ' << ref.time << ' ' << ref.pose.x << ' '
                  << ref.pose.y << ' ' << ref.pose.yaw << ' ' << ref.twist.vx << ' '
                  << ref.twist.vy << ' ' << ref.twist.wz;
      } else {
        std::cout << "-1 0 0 0 0 0 0 0";
      }
      for (std::size_t j = 0; j < out.reasons.count; ++j) {
        std::cout << ' ' << out.reasons.codes[j];
      }
      std::cout << '\n';
    }
  }
  return std::cin.eof() ? 0 : 1;
}
