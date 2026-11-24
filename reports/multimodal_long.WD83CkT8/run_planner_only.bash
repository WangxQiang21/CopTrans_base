#!/usr/bin/env bash
set -eo pipefail

repository=/home/wq/Whut/CopTrans_base
results=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
scenario=$1
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="$2" ROS_LOG_DIR="$results/$scenario.ros"
cd "$repository"
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
export PYTHONPATH="/home/wq/Whut/.coptrans-python${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1

./scripts/test.sh --task multi --scenario "$scenario" --experiment full_route --dry-run > "$results/$scenario.config.yaml"
parameters="$results/$scenario.planner.yaml"
python3 - "$results/$scenario.config.yaml" "$parameters" <<'PY'
from pathlib import Path
import sys
import yaml
from fwis_bringup.test_config import cargo_footprint_parameters
from ranger_mini_gz.scene_generator import load_scene

configuration = yaml.safe_load(Path(sys.argv[1]).read_text())
assert configuration["coordination"]["implementation"] == "uniform_v02"
scene = load_scene(configuration["scenario"])
parameters = configuration["planner_overrides"]
parameters["cargo_footprint"] = cargo_footprint_parameters(scene.cargo.size, scene.cargo.offset)
Path(sys.argv[2]).write_text(yaml.safe_dump(parameters, sort_keys=False))
PY

env --default-signal=INT,TERM setsid ros2 launch --noninteractive fwis_bringup plan.launch.py \
    backend:=multi_4wis transport_ui:=false rviz:=false overrides_file:="$parameters" \
    > "$results/$scenario.launch.log" 2>&1 &
planner_pid=$!
trap 'kill -INT "$planner_pid" 2>/dev/null || true; wait "$planner_pid" 2>/dev/null || true' EXIT
timeout 165s python3 ros2_ws/src/fwis_bringup/test/probe_scene_plan.py "$scenario" formation \
    --planner-only --output "$results/$scenario.json" > "$results/$scenario.probe.log" 2>&1
