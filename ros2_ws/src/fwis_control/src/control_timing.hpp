#ifndef FWIS_CONTROL_CONTROL_TIMING_HPP_
#define FWIS_CONTROL_CONTROL_TIMING_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <utility>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/timer.hpp>
#include <sys/resource.h>
#include <time.h>

namespace fwis_control
{

using WallClock = std::chrono::steady_clock;

inline int64_t steady_ns(WallClock::time_point point = WallClock::now()) noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(point.time_since_epoch()).count();
}

inline int64_t realtime_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

struct TimerDispatch
{
  int64_t due_clock_ns{};
  int64_t called_clock_ns{};
  int64_t called_steady_ns{};
  int64_t period_ns{};
};

struct OdometryTiming
{
  int64_t odom_stamp_ns{};
  int64_t source_realtime_ns{};
  int64_t received_realtime_ns{};
  int64_t callback_realtime_ns{};
  int64_t callback_steady_ns{};
};

struct ThreadCounters
{
  int64_t cpu_ns{};
  int64_t voluntary_switches{};
  int64_t involuntary_switches{};
};

inline ThreadCounters thread_counters() noexcept
{
  timespec cpu{};
  rusage usage{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu) != 0 ||
    getrusage(RUSAGE_THREAD, &usage) != 0)
  {
    return {-1, -1, -1};
  }
  return {cpu.tv_sec * 1000000000LL + cpu.tv_nsec, usage.ru_nvcsw, usage.ru_nivcsw};
}

inline ThreadCounters counter_delta(const ThreadCounters & start, const ThreadCounters & end) noexcept
{
  if (start.cpu_ns < 0 || end.cpu_ns < 0) {
    return {-1, -1, -1};
  }
  return {end.cpu_ns - start.cpu_ns, end.voluntary_switches - start.voluntary_switches,
    end.involuntary_switches - start.involuntary_switches};
}

struct ControlTiming
{
  TimerDispatch timer;
  OdometryTiming odom;
  ThreadCounters thread_at_entry;
  ThreadCounters publish_thread;
  ThreadCounters callback_thread;
  int64_t started_steady_ns{};
  int64_t ended_steady_ns{};
  int64_t ended_clock_ns{};
  int64_t core_ns{};
  int64_t command_publish_steady_ns{};
  int64_t command_published_steady_ns{};
  uint64_t status_drops{};
};

// Humble's GenericTimer does not expose the expected call time to the
// callback. Take the scheduled offset immediately before rcl_timer_call,
// using the same clock and timer as the production GenericTimer.
class TimingTimer final : public rclcpp::TimerBase
{
public:
  TimingTimer(
    rclcpp::Clock::SharedPtr clock, std::chrono::nanoseconds period,
    rclcpp::Context::SharedPtr context, std::function<void(TimerDispatch)> callback)
  : rclcpp::TimerBase(std::move(clock), period, std::move(context)),
    period_ns_(period.count()), callback_(std::move(callback))
  {}

  bool call() override
  {
    const auto handle = get_timer_handle();
    const auto until = time_until_trigger();
    if (until == std::chrono::nanoseconds::max()) {
      return false;
    }
    const auto called_clock = clock_->now().nanoseconds();
    dispatch_ = {called_clock + until.count(), called_clock, steady_ns(), period_ns_};
    const auto result = rcl_timer_call(const_cast<rcl_timer_t *>(handle.get()));
    if (result == RCL_RET_TIMER_CANCELED) {
      return false;
    }
    if (result != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(result, "control timing timer could not advance");
    }
    return true;
  }

  void execute_callback() override {callback_(dispatch_);}

  bool is_steady() override {return clock_->get_clock_type() == RCL_STEADY_TIME;}

private:
  int64_t period_ns_;
  std::function<void(TimerDispatch)> callback_;
  TimerDispatch dispatch_;
};

}  // namespace fwis_control
#endif  // FWIS_CONTROL_CONTROL_TIMING_HPP_
