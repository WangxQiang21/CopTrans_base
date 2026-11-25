"""Opt-in nonblocking event capture for the normal Gazebo body transport."""

from collections import deque
import csv
from pathlib import Path
from resource import RUSAGE_THREAD, getrusage
from threading import Thread, get_native_id
from time import sleep, thread_time_ns


HEADER = ("event", "identity", "observed_steady_ns", "consume_steady_ns",
          "applied_steady_ns", "vx_mps", "vy_mps", "wz_radps", "previous_steady_ns",
          "sim_advance_ns", "thread_cpu_delta_ns", "voluntary_switches",
          "involuntary_switches")
SLOW_STEP_NS = 10_000_000


class PlantTiming:
    def __init__(self, directory, model_name):
        self.events = deque(maxlen=8192)
        self.sequence = 0
        self.last_consumed = 0
        self.previous_step = None
        path = Path(directory) / f"{model_name}_plant_timing.csv"
        Thread(target=write_events, args=(path, self.events), daemon=True,
               name=f"{model_name}-plant-timing").start()

    def odometry(self, sim_stamp_ns, published_ns):
        self.events.append(("odometry", sim_stamp_ns, published_ns, "", "", "", "", ""))

    def consumed(self, identity, received_ns, consumed_ns, applied_ns, command):
        self.last_consumed = identity
        self.events.append(("command", identity, received_ns, consumed_ns, applied_ns, *command))

    def physics_step(self, sim_ns, wall_ns):
        thread = get_native_id()
        usage = getrusage(RUSAGE_THREAD)
        cpu_ns = thread_time_ns()
        previous = self.previous_step
        self.previous_step = thread, sim_ns, wall_ns, cpu_ns, usage.ru_nvcsw, usage.ru_nivcsw
        if previous is None or previous[0] != thread or wall_ns - previous[2] < SLOW_STEP_NS:
            return
        _, previous_sim, previous_wall, previous_cpu, previous_voluntary, previous_involuntary = previous
        self.events.append(("physics_gap", sim_ns, wall_ns, "", "", "", "", "", previous_wall,
                            sim_ns - previous_sim, cpu_ns - previous_cpu,
                            usage.ru_nvcsw - previous_voluntary, usage.ru_nivcsw - previous_involuntary))


def write_events(path, events):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        while True:
            # The Gazebo callback only appends immutable records. Flush regularly
            # so the usual process shutdown keeps all but the final polling window.
            while events:
                writer.writerow(events.popleft())
            stream.flush()
            sleep(.10)
