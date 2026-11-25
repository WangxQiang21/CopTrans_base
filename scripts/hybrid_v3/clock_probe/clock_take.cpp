// Diagnostic-only RCL interposition for the native controller. Observe the samples
// consumed by the existing Humble TimeSource and odometry subscription.
#include <rcl/subscription.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <dlfcn.h>
#include <sys/resource.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

namespace
{
using Nanoseconds = int64_t;

struct Sample
{
  char topic;
  Nanoseconds steady, realtime, source, received, stamp, cpu;
  long voluntary, involuntary;
};

// At 500 Hz clock and 50 Hz odometry the buffer covers the entire 890 s
// qualification timeout. Log overflow explicitly if the cadence changes.
std::array<Sample, 600000> samples;
std::atomic<unsigned> count{0};

Nanoseconds now_ns(clockid_t clock)
{
  timespec time{};
  clock_gettime(clock, &time);
  return time.tv_sec * 1000000000LL + time.tv_nsec;
}

template<class Stamp>
Nanoseconds stamp_ns(const Stamp & stamp)
{
  return static_cast<Nanoseconds>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool get_robot_name(char (&name)[7])
{
  auto * file = std::fopen("/proc/self/cmdline", "rb");
  if (!file) {
    return false;
  }
  char args[8192];
  const auto bytes = std::fread(args, 1, sizeof(args), file);
  std::fclose(file);
  constexpr char ns[] = "__ns:=/robot";
  for (size_t i = 0; i + sizeof(ns) < bytes; ++i) {
    if (std::memcmp(args + i, ns, sizeof(ns) - 1) == 0) {
      const char id = args[i + sizeof(ns) - 1];
      if (id >= '1' && id <= '3' && args[i + sizeof(ns)] == '\0') {
        std::snprintf(name, sizeof(name), "robot%c", id);
        return true;
      }
    }
  }
  return false;
}

__attribute__((destructor)) void write_samples()
{
  const auto * directory = std::getenv("COPTRANS_TIMING_DIR");
  char robot[7]{};
  if (!directory || count.load() == 0 || !get_robot_name(robot)) {
    return;
  }
  char path[4096];
  std::snprintf(path, sizeof(path), "%s/%s_clock_take.csv", directory, robot);
  auto * file = std::fopen(path, "w");
  if (!file) {
    return;
  }
  std::fputs("topic,take_steady_ns,take_realtime_ns,source_realtime_ns,received_realtime_ns,"
    "stamp_ns,thread_cpu_ns,voluntary_switches,involuntary_switches,overflow\n", file);
  const auto total = count.load();
  const auto saved = std::min<unsigned>(total, samples.size());
  for (unsigned i = 0; i < saved; ++i) {
    const auto & s = samples[i];
    std::fprintf(file, "%c,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u\n", s.topic, s.steady,
      s.realtime, s.source, s.received, s.stamp, s.cpu, s.voluntary, s.involuntary, total - saved);
  }
  std::fclose(file);
}
}  // namespace

extern "C" rcl_ret_t rcl_take(const rcl_subscription_t * subscription, void * message,
  rmw_message_info_t * info, rmw_subscription_allocation_t * allocation)
{
  using Take = decltype(&rcl_take);
  static const auto take = reinterpret_cast<Take>(dlsym(RTLD_NEXT, "rcl_take"));
  if (!take) {
    return RCL_RET_ERROR;
  }
  const auto result = take(subscription, message, info, allocation);
  if (result != RCL_RET_OK || !info) {
    return result;
  }
  const auto * topic = rcl_subscription_get_topic_name(subscription);
  if (!topic) {
    return result;
  }
  const char * name = std::strrchr(topic, '/');
  char kind;
  Nanoseconds stamp;
  if (std::strcmp(topic, "/clock") == 0) {
    kind = 'C';
    stamp = stamp_ns(static_cast<rosgraph_msgs::msg::Clock *>(message)->clock);
  } else if (name && std::strcmp(name, "/odom") == 0) {
    kind = 'O';
    stamp = stamp_ns(static_cast<nav_msgs::msg::Odometry *>(message)->header.stamp);
  } else {
    return result;
  }
  const auto index = count.fetch_add(1, std::memory_order_relaxed);
  if (index < samples.size()) {
    const auto steady = now_ns(CLOCK_MONOTONIC);
    const auto realtime = now_ns(CLOCK_REALTIME);
    const auto cpu = now_ns(CLOCK_THREAD_CPUTIME_ID);
    rusage usage{};
    getrusage(RUSAGE_THREAD, &usage);
    samples[index] = {kind, steady, realtime, info->source_timestamp, info->received_timestamp,
      stamp, cpu, usage.ru_nvcsw, usage.ru_nivcsw};
  }
  return result;
}
