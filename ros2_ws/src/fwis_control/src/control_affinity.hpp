#ifndef FWIS_CONTROL_CONTROL_AFFINITY_HPP_
#define FWIS_CONTROL_CONTROL_AFFINITY_HPP_

#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include <sched.h>

namespace fwis_control
{

// Middleware threads inherit the launching thread's mask at creation. On a
// reserved SMT core, leave one logical CPU exclusively for its control executor.
inline int reserve_control_cpu()
{
  const char * split = std::getenv("COPTRANS_SPLIT_TRACKER_CORE");
  cpu_set_t mask;
  if (!split || std::strcmp(split, "1") != 0 ||
    sched_getaffinity(0, sizeof(mask), &mask) != 0 || CPU_COUNT(&mask) < 2)
  {
    return -1;
  }
  int cpu = 0;
  const char * requested = std::getenv("COPTRANS_CONTROL_WORKER_CPU");
  if (requested) {
    char * end = nullptr;
    const long value = std::strtol(requested, &end, 10);
    if (end == requested || *end || value < 0 || value >= CPU_SETSIZE ||
      !CPU_ISSET(static_cast<int>(value), &mask))
    {
      return -1;
    }
    cpu = static_cast<int>(value);
  } else {
    while (!CPU_ISSET(cpu, &mask)) {
      ++cpu;
    }
  }
  CPU_CLR(cpu, &mask);
  return sched_setaffinity(0, sizeof(mask), &mask) == 0 ? cpu : -1;
}

inline void bind_control_cpu(int cpu)
{
  if (cpu < 0) {
    return;
  }
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(cpu, &mask);
  if (sched_setaffinity(0, sizeof(mask), &mask) != 0) {
    throw std::runtime_error("could not bind the native control executor to its reserved CPU");
  }
}

}  // namespace fwis_control

#endif  // FWIS_CONTROL_CONTROL_AFFINITY_HPP_
