// Opt-in capture of the successful ROS command take in the existing ros_gz_bridge process.
#include "bridge_take_store.hpp"
#include <chrono>
#include <cstdlib>
#include <string_view>

#include <dlfcn.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl/subscription.h>

namespace
{

bridge_take_trace::Store store;

// Open and pre-fault the diagnostic journal outside the DDS take path.
__attribute__((constructor)) void open_trace()
{
  store.open(std::getenv("COPTRANS_TIMING_DIR"));
}

template<typename Clock>
int64_t now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    Clock::now().time_since_epoch()).count();
}

// The qualification runner serializes the shared journal only after the
// bridge has stopped. No successful take depends on normal process teardown.
__attribute__((destructor)) void close_trace()
{
  store.finish();
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
  const auto & command = *static_cast<const geometry_msgs::msg::Twist *>(message);
  store.record({topic[6]-'0', observed, realtime, info->source_timestamp, info->received_timestamp,
    command.linear.x, command.linear.y, command.angular.z});
  return result;
}
