"""Bounded diagnostic sampling for local closed-loop experiments."""

from collections import deque
import json


TREND_S = .5
FAULT_WINDOW_S = 8.
MAX_FAULT_FRAMES = 1000


class ControllerLog:
    def __init__(self, directory, *, compact):
        self.directory = directory
        self.compact = compact
        self.stream = (directory / "controller.log").open("w", buffering=1)
        self.previous = {}
        self.tail = deque(maxlen=MAX_FAULT_FRAMES)

    def record(self, vehicle, row, wall):
        serialized = json.dumps(dict(row, vehicle_id=vehicle), ensure_ascii=False) + "\n"
        if not self.compact:
            self.stream.write(serialized)
            return
        self.tail.append((wall, serialized))
        state = tuple(row.get(field) for field in ("kind", "phase", "result", "planned_mode", "feedback_mode",
                                                   "requested_mode", "reasons"))
        stamp, old = self.previous.get(vehicle, (-float("inf"), None))
        if state != old or wall - stamp >= TREND_S:
            self.stream.write(serialized)
            self.previous[vehicle] = wall, state

    def save_fault_tail(self, wall):
        if self.compact and self.tail:
            (self.directory / "metrics" / "fault_tail.jsonl").write_text(
                "".join(payload for stamp, payload in self.tail if wall - stamp <= FAULT_WINDOW_S))
