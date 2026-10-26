"""Versioned environment settings shared by diagnostics, data and training."""
from dataclasses import asdict, dataclass, fields
from pathlib import Path
import math

import yaml

from .action_library import ActionLibrary
from .environment import EnvironmentConfig
from .kernel_adapter import PROJECT_ROOT, FinalizationConfig, load_contract, stable_hash


@dataclass(frozen=True)
class Settings:
    contract: object
    library: object
    environment: EnvironmentConfig
    finalization: FinalizationConfig
    problem_kwargs: dict
    source: dict

    @property
    def identity(self):
        return stable_hash({"contract": self.contract.identity,
                            "library": self.library.identity,
                            "environment": asdict(self.environment),
                            "finalization": asdict(self.finalization),
                            "problem_kwargs": self.problem_kwargs})


def load_settings(path=None):
    path = Path(path) if path else PROJECT_ROOT/"rl/configs/environment.yaml"
    source = yaml.safe_load(path.read_text())
    if source.get("schema_version") != "aos_environment_v1":
        raise ValueError("unsupported_environment_schema")
    if source.get("action_library") != "aos_80_v1":
        raise ValueError("unsupported_action_library")
    common_vehicle = PROJECT_ROOT/"catkin_ws/src/fwis_common/config/ranger_mini_2.yaml"
    if (PROJECT_ROOT/source["vehicle_config"]).resolve() != common_vehicle.resolve():
        raise ValueError("environment_requires_common_vehicle_config")
    direction = source["direction_policy"]
    if direction not in ("bidirectional", "forward_only"):
        raise ValueError("invalid_direction_policy")
    if source["initial_mode"] != 0:
        raise ValueError("unsupported_initial_mode")
    if not isinstance(source["allow_shunting"], bool):
        raise ValueError("allow_shunting_must_be_boolean")
    xy, yaw = float(source["goal_xy_tol_m"]), math.radians(float(source["goal_yaw_tol_deg"]))
    if not all(math.isfinite(v) and 0 < v <= cap for v, cap in ((xy, .02), (yaw, math.pi/180))):
        raise ValueError("goal_tolerance_exceeds_common_contract")
    contract = load_contract(PROJECT_ROOT/source["planning_config"])
    names = {f.name for f in fields(EnvironmentConfig)}
    environment = EnvironmentConfig(**{key: value for key, value in source.items() if key in names})
    finalization = FinalizationConfig(sweep_spacing=float(source["mask_max_corner_step_m"]),
                                      include_metrics=False)
    problem_kwargs = {"allow_reverse": direction == "bidirectional",
                      "allow_shunting": source["allow_shunting"], "initial_mode": 0,
                      "xy_tol": xy, "yaw_tol": yaw}
    return Settings(contract, ActionLibrary.from_contract(contract), environment,
                    finalization, problem_kwargs, source)
