"""Read a shared controller profile for launch and repeatable comparisons."""
from dataclasses import dataclass
from pathlib import Path

import yaml

from .controllers import NonlinearFeedback, NonlinearGains, PIDFeedback, PIDGains
from .execution import Execution, Timing
from .safety import Limits
from .trajectory import PreflightSettings


@dataclass(frozen=True)
class Configuration:
    limits: Limits
    timing: Timing
    preflight: PreflightSettings
    nonlinear: NonlinearGains
    pid: dict

    def execution(self, algorithm):
        if algorithm == "pid":
            controller = PIDFeedback(self.pid)
        elif algorithm == "nonlinear":
            controller = NonlinearFeedback(self.nonlinear)
        else:
            raise ValueError(f"unknown tracking algorithm: {algorithm}")
        return Execution(controller, self.limits, self.preflight, self.timing)


def load(path):
    data = yaml.safe_load(Path(path).read_text())
    pid = {key: PIDGains(**parameters) for key, parameters in data["pid"].items()}
    if set(pid) != {"ack_x", "ack_y", "ack_yaw", "oblique", "spin"}:
        raise ValueError("missing or unknown mode-specific PID gains")
    return Configuration(Limits(**data["limits"]), Timing(**data["timing"]),
                         PreflightSettings(**data["preflight"]), NonlinearGains(**data["nonlinear"]), pid)
