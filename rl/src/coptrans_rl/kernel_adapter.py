"""Single import/configuration boundary to the existing ROS-free kernel."""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
KERNEL_SOURCE = PROJECT_ROOT/"catkin_ws/src/fwis_v3_optimizer/src"
if str(KERNEL_SOURCE) not in sys.path:
    sys.path.insert(0, str(KERNEL_SOURCE))

from fwis_v3_optimizer.core import Action, Grid, Pose, advance, replay, wrap
from fwis_v3_optimizer.retiming import MotionLimits
from fwis_v3_optimizer.quality import QualityProfile, action_shunting_violation
from fwis_v3_optimizer.candidate_finalizer import (
    PlanningProblem, FinalizationConfig, finalize_candidate,
    finalize_candidate_set)


def stable_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class VehicleContract:
    length: float
    width: float
    margin: float
    limits: MotionLimits
    quality: QualityProfile

    @property
    def identity(self):
        return stable_hash(asdict(self))

    def problem(self, grid, start, goal, **kwargs):
        if (not math.isclose(2*grid.half_l, self.length+self.margin, abs_tol=1e-10) or
                not math.isclose(2*grid.half_w, self.width+self.margin, abs_tol=1e-10)):
            raise ValueError("grid_vehicle_contract_mismatch")
        return PlanningProblem(grid, start, goal, self.limits, self.length,
                               self.width, quality_profile=self.quality, **kwargs)


def load_contract(config_file=None):
    vehicle_path = PROJECT_ROOT/"catkin_ws/src/fwis_common/config/ranger_mini_2.yaml"
    config_path = (Path(config_file) if config_file else
                   PROJECT_ROOT/"catkin_ws/src/fwis_v3_demo/config/params.yaml")
    vehicle = yaml.safe_load(vehicle_path.read_text())["vehicle"]
    config = yaml.safe_load(config_path.read_text())
    vehicle.update(config.get("vehicle", {}))
    limits = dict(config.get("quality", {}).get("limits", {}))
    for name in ("wheelbase", "track", "r_min_ack", "mode_switch_duration",
                 "mode_setup_time"):
        limits[name] = vehicle[name]
    return VehicleContract(float(vehicle["length"]), float(vehicle["width"]),
                           float(config.get("collision", {}).get("margin", .04)),
                           MotionLimits(**limits),
                           QualityProfile(**config.get("quality", {}).get("cost", {})))
