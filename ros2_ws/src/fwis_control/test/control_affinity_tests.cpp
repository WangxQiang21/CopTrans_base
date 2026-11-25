#include "../src/control_affinity.hpp"
#include "../src/control_timing.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

int main()
{
  cpu_set_t original;
  if (sched_getaffinity(0, sizeof(original), &original) != 0) {
    return 1;
  }
  unsetenv("COPTRANS_SPLIT_TRACKER_CORE");
  if (fwis_control::reserve_control_cpu() != -1) {
    return 1;
  }
  if (CPU_COUNT(&original) < 2) {
    return 77;
  }
  setenv("COPTRANS_SPLIT_TRACKER_CORE", "1", 1);
  // The worker lane may sit in the middle of the wider middleware pool.
  int expected = CPU_SETSIZE - 1;
  while (!CPU_ISSET(expected, &original)) {
    --expected;
  }
  setenv("COPTRANS_CONTROL_WORKER_CPU", "invalid", 1);
  if (fwis_control::reserve_control_cpu() != -1) {
    return 1;
  }
  setenv("COPTRANS_CONTROL_WORKER_CPU", std::to_string(expected).c_str(), 1);
  const int chosen = fwis_control::reserve_control_cpu();
  if (chosen != expected) {
    return 1;
  }
  cpu_set_t helpers;
  if (sched_getaffinity(0, sizeof(helpers), &helpers) != 0 ||
    CPU_ISSET(chosen, &helpers) || CPU_COUNT(&helpers) != CPU_COUNT(&original) - 1)
  {
    return 1;
  }
  bool passed = false;
  std::thread worker([&]() {
    fwis_control::bind_control_cpu(chosen);
    cpu_set_t mask;
    if (sched_getaffinity(0, sizeof(mask), &mask) != 0 ||
      CPU_COUNT(&mask) != 1 || !CPU_ISSET(chosen, &mask))
    {
      return;
    }
    // The same monotonic, per-thread counters used to distinguish a blocking
    // call from an involuntary preemption. Sleeping must include a voluntary yield.
    const auto before = fwis_control::thread_counters();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const auto delta = fwis_control::counter_delta(before, fwis_control::thread_counters());
    const auto invalid = fwis_control::counter_delta({-1, -1, -1}, before);
    passed = delta.cpu_ns >= 0 && delta.voluntary_switches > 0 &&
      delta.involuntary_switches >= 0 && invalid.cpu_ns == -1 &&
      fwis_control::reserve_control_cpu() == -1;
  });
  worker.join();
  cpu_set_t final_mask;
  if (sched_getaffinity(0, sizeof(final_mask), &final_mask) != 0 ||
    !CPU_EQUAL(&helpers, &final_mask) || !passed)
  {
    std::cerr << "control and middleware thread masks or counters differ from the contract\n";
    return 1;
  }
  if (sched_setaffinity(0, sizeof(original), &original) != 0) {
    return 1;
  }
  unsetenv("COPTRANS_CONTROL_WORKER_CPU");
  return fwis_control::reserve_control_cpu() >= 0 ? 0 : 1;
}
