"""Opt-in nonblocking event capture for the normal Gazebo body transport."""

from collections import deque
import csv
from pathlib import Path
from threading import Thread
from time import sleep


HEADER = ("event", "identity", "observed_steady_ns", "consume_steady_ns",
          "applied_steady_ns", "vx_mps", "vy_mps", "wz_radps")


class PlantTiming:
    def __init__(self, directory, model_name):
        self.events = deque(maxlen=8192)
        self.sequence = 0
        self.last_consumed = 0
        path = Path(directory) / f"{model_name}_plant_timing.csv"
        Thread(target=write_events, args=(path, self.events), daemon=True,
               name=f"{model_name}-plant-timing").start()

    def odometry(self, sim_stamp_ns, published_ns):
        self.events.append(("odometry", sim_stamp_ns, published_ns, "", "", "", "", ""))

    def consumed(self, identity, received_ns, consumed_ns, applied_ns, command):
        self.last_consumed = identity
        self.events.append(("command", identity, received_ns, consumed_ns, applied_ns, *command))


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
