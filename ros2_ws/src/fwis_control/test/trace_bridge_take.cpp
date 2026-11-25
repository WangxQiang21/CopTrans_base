// Opt-in capture of the successful ROS command take in the existing ros_gz_bridge process.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include <dlfcn.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl/subscription.h>

namespace
{

struct Take
{
  char robot;
  int64_t steady_ns, realtime_ns, source_ns, received_ns;
  double vx, vy, wz;
};

std::array<Take, 65'536> trace;
std::atomic<size_t> count{0};

template<typename Clock>
int64_t now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    Clock::now().time_since_epoch()).count();
}

// Flush after the bridge exits its application loop. No allocation or file I/O occurs at take.
__attribute__((destructor)) void write_trace()
{
  const char * directory = std::getenv("COPTRANS_TIMING_DIR");
  const size_t size = count.load(std::memory_order_relaxed);
  if (!directory || size == 0) {
    return;
  }
  char path[4096];
  const int length = std::snprintf(path, sizeof(path), "%s/bridge_take_%d.csv", directory, getpid());
  if (length < 0 || static_cast<size_t>(length) >= sizeof(path)) {
    return;
  }
  auto * output = std::fopen(path, "w");
  if (!output) {
    return;
  }
  std::fputs("robot,observed_steady_ns,taken_realtime_ns,source_realtime_ns,received_realtime_ns,"
    "vx_mps,vy_mps,wz_radps,trace_overflow\n", output);
  const size_t available = std::min(size, trace.size());
  for (size_t index = 0; index < available; ++index) {
    const auto & row = trace[index];
    std::fprintf(output, "robot%c,%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64
      ",%.17g,%.17g,%.17g,%zu\n", row.robot, row.steady_ns, row.realtime_ns, row.source_ns,
      row.received_ns, row.vx, row.vy, row.wz, size - available);
  }
  std::fclose(output);
}

}  // namespace

extern "C" rcl_ret_t rcl_take(
  const rcl_subscription_t * subscription, void * message, rmw_message_info_t * info,
  rmw_subscription_allocation_t * allocation)
{
  static const auto take = reinterpret_cast<decltype(&rcl_take)>(dlsym(RTLD_NEXT, "rcl_take"));
  if (!take) {
    return RCL_RET_ERROR;
  }
  const auto result = take(subscription, message, info, allocation);
  if (result != RCL_RET_OK || !info) {
    return result;
  }
  const char * name = rcl_subscription_get_topic_name(subscription);
  const std::string_view topic = name ? name : "";
  if (topic != "/robot1/cmd_vel" && topic != "/robot2/cmd_vel" && topic != "/robot3/cmd_vel") {
    return result;
  }
  const auto observed = now_ns<std::chrono::steady_clock>();
  const auto realtime = now_ns<std::chrono::system_clock>();
  const size_t index = count.fetch_add(1, std::memory_order_relaxed);
  if (index < trace.size()) {
    const auto & command = *static_cast<const geometry_msgs::msg::Twist *>(message);
    trace[index] = {topic[6], observed, realtime, info->source_timestamp, info->received_timestamp,
      command.linear.x, command.linear.y, command.angular.z};
  }
  return result;
}
