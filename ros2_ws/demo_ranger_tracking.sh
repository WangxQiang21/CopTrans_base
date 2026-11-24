#!/usr/bin/env bash
set -e

workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -r "$workspace/install/local_setup.bash" ]]; then
    echo "ROS 2 工作空间还未构建。请先按 ros2_ws/README.md 构建。" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$workspace/install/local_setup.bash"
exec python3 -u "$workspace/src/fwis_control/test/demo_tracking.py" "$@"
