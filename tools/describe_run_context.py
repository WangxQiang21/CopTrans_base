#!/usr/bin/env python3
"""Print effective planner, map, and source fingerprints before ROS launch."""
import hashlib
import math
import sys
from pathlib import Path

import yaml


def main(argv):
    if len(argv) < 4:
        raise SystemExit("usage: describe_run_context.py ROOT MAP_DIR SCALE [ROS_ARGS...]")
    demo = Path(argv[1]).resolve()
    map_dir = Path(argv[2]).resolve() if argv[2] else None
    scale = argv[3]
    args = argv[4:]
    config = demo / "catkin_ws/src/fwis_v3_demo/config/params.yaml"
    backend_override = None
    target_override = None
    config_override = None
    direction_override = None
    legacy_reverse_override = None
    for arg in args:
        if arg.startswith("planning_backend:="):
            backend_override = arg.split(":=", 1)[1]
        elif arg.startswith("target_v:="):
            target_override = float(arg.split(":=", 1)[1])
            if not math.isfinite(target_override) or target_override <= 0:
                raise SystemExit("target_v must be a finite positive value")
        elif arg.startswith("config_file:="):
            config_override = arg.split(":=", 1)[1]
        elif arg.startswith("direction_policy:="):
            direction_override = arg.split(":=", 1)[1]
        elif arg.startswith("allow_reverse_fallback:="):
            raw = arg.split(":=", 1)[1].lower()
            if raw not in ("true", "false"):
                raise SystemExit("allow_reverse_fallback must be true or false")
            legacy_reverse_override = raw == "true"
    if config_override:
        config = Path(config_override).expanduser().resolve()
    params = yaml.safe_load(config.read_text(encoding="utf-8"))
    planning = params.get("planning", {})
    search = params.get("search", {})
    vehicle = params.get("vehicle", {})
    backend = backend_override or planning.get("backend", "legacy")
    direction = direction_override or planning.get("direction_policy")
    legacy_reverse = (legacy_reverse_override if legacy_reverse_override is not None
                      else planning.get("allow_reverse_fallback"))
    if direction_override is None and legacy_reverse_override is not None:
        direction = ("bidirectional" if legacy_reverse_override else "forward_only")
    if direction is not None and direction not in ("bidirectional", "forward_only"):
        raise SystemExit("unknown direction_policy: %s" % direction)
    if legacy_reverse is not None and not isinstance(legacy_reverse, bool):
        raise SystemExit("allow_reverse_fallback in config must be boolean")
    if direction is not None and legacy_reverse is not None:
        legacy_policy = "bidirectional" if legacy_reverse else "forward_only"
        if direction != legacy_policy:
            raise SystemExit("conflicting direction_policy and allow_reverse_fallback")
    if direction is None and legacy_reverse is not None:
        direction = "bidirectional" if legacy_reverse else "forward_only"
    if backend in ("mode_path_v1", "mode_path_v2"):
        if direction is None:
            direction = "bidirectional"
        direction = ("bidirectional_costed_candidates" if
                     direction == "bidirectional" else "forward_only")
    else:
        direction = "legacy_historical_policy"
    print("[expv3] run_contract backend=%s direction=%s vehicle_m=%sx%s "
          "coarse_goal_tol_m=%s yaw_rad=%s map_scale=%s" %
          (backend, direction, vehicle.get("length", "?"),
           vehicle.get("width", "?"), search.get("goal_xy_tol", "?"),
           search.get("goal_yaw_tol", "?"), scale))
    limits = params.get("quality", {}).get("limits", {})
    target = target_override if target_override is not None else limits.get("target_v", limits.get("v_max"))
    print("[expv3] timing_policy=%s mode_switch_s=%s" % (
        limits.get("translation_speed_policy", "restricted"),
        limits.get("mode_switch_duration", "steering_limits_plus_setup")))
    print("[expv3] speed_contract target_v_mps=%s lateral_a_max_mps2=%s spin_omega_max=%s "
          "translation_omega_max=%s reverse_preference=late_soft" %
          (target, limits.get("lateral_a_max"), limits.get("omega_max"),
           limits.get("translation_omega_max", limits.get("omega_max"))))
    if map_dir and (map_dir / "map.yaml").is_file():
        metadata = yaml.safe_load((map_dir / "map.yaml").read_text(encoding="utf-8"))
        image = (map_dir / metadata["image"]).resolve()
        digest = hashlib.sha256()
        for file in (map_dir / "map.yaml", image):
            digest.update(file.name.encode("utf-8"))
            digest.update(file.read_bytes())
        print("[expv3] map_dir=%s map_sha256=%s" % (map_dir, digest.hexdigest()))
    files = [
        demo / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/retiming.py",
        demo / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_geometry.py",
        demo / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_retiming.py",
        demo / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/quality.py",
        demo / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/mode_master.py",
        demo / "catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py",
        demo / "catkin_ws/src/fwis_v3_planner/src/planner_node.cpp",
    ]
    digest = hashlib.sha256()
    for file in files:
        content = hashlib.sha256(file.read_bytes()).hexdigest()
        digest.update(file.relative_to(demo).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(content.encode("ascii"))
        digest.update(b"\n")
    print("[expv3] planning_source_sha256=%s" % digest.hexdigest())
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
