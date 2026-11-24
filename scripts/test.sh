#!/usr/bin/env bash
set -e -o pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace="$root/ros2_ws"
temporary=$(mktemp "${TMPDIR:-/tmp}/coptrans-test.XXXXXXXX")
run_dir=
launch_pid=
interrupted=

stop_remaining() {
    local attempt
    if [[ -z $launch_pid ]] || ! kill -0 -- "-$launch_pid" 2>/dev/null; then
        return
    fi
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    for ((attempt=0; attempt<20; attempt++)); do
        kill -0 -- "-$launch_pid" 2>/dev/null || return 0
        sleep .1
    done
    kill -KILL -- "-$launch_pid" 2>/dev/null || true
}

cleanup() {
    local code=$?
    rm -f -- "$temporary"
    stop_remaining
    if [[ -n $run_dir && ! -f $run_dir/result.yaml ]]; then
        if [[ -n $interrupted ]]; then
            printf 'status: cancelled\nreason: operator_interrupt\nexit_code: %s\n' "$code" >"$run_dir/result.yaml"
        else
            printf 'status: launch_failed\nreason: startup_or_launch_exited\nexit_code: %s\n' "$code" >"$run_dir/result.yaml"
        fi
    fi
}

interrupt() {
    interrupted=$1
    if [[ -n $launch_pid ]]; then
        # Humble Launch forwards SIGINT to its nodes and awaits their exit.
        kill -INT "$launch_pid" 2>/dev/null || true
    else
        exit "$2"
    fi
}

trap cleanup EXIT
trap 'interrupt INT 130' INT
trap 'interrupt TERM 143' TERM

COPTRANS_CONFIG_OUT="$temporary" PYTHONPATH="$workspace/src/ranger_mini_gz${PYTHONPATH:+:$PYTHONPATH}" \
    python3 "$workspace/src/fwis_bringup/fwis_bringup/test_config.py" "$@"
[[ -s $temporary ]] || exit 0

mapfile -t config < <(python3 - "$temporary" <<'PY'
import json, sys
with open(sys.argv[1]) as handle:
    config = json.load(handle)
for key in ("task", "scenario", "planner", "no_build", "record"):
    print(config[key])
PY
)

run_root="$root/reports/runs"
mkdir -p -- "$run_root"
exec 9>"$run_root/.test.lock"
if ! flock -n 9; then
    printf 'ERROR: another CopTrans test is already running.\n' >&2
    exit 2
fi
run_dir=$(mktemp -d "$run_root/$(date +%Y%m%d_%H%M%S)_${config[0]}_${config[1]}_${config[2]}_XXXXXX")
mkdir -p -- "$run_dir"/{metrics,trajectories,screenshots,ros}
{
    printf '%q' "$root/scripts/test.sh"
    if (($#)); then
        printf ' %q' "$@"
    fi
    printf '\n'
} >"$run_dir/command.txt"
cp -- "$temporary" "$run_dir/resolved_config.yaml"
printf 'Run directory %s\n' "$run_dir"

fail() {
    printf 'ERROR: %s\n' "$1" >&2
    printf 'status: launch_failed\nreason: %s\nexit_code: 2\n' "$2" >"$run_dir/result.yaml"
    exit 2
}

[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble setup is missing.' ros_setup_missing
[[ -n ${DISPLAY:-} || -n ${WAYLAND_DISPLAY:-} ]] || fail 'Gazebo and RViz2 require a graphical display.' display_missing
command -v gz >/dev/null || fail 'Gazebo Harmonic command gz is missing.' gazebo_missing
gazebo_version=$(gz sim --versions) || fail 'Gazebo Sim could not report its version.' gazebo_missing
[[ $gazebo_version =~ (^|[[:space:]])8\. ]] || fail 'Gazebo Sim 8 (Harmonic) is required.' gazebo_version_mismatch
source /opt/ros/humble/setup.bash

export ROS_LOG_DIR="$run_dir/ros" ROS_LOCALHOST_ONLY=1
# Use an unused local DDS port range below the normal Linux ephemeral range.
if ! ROS_DOMAIN_ID=$(python3 - <<'PY'
from pathlib import Path
from random import SystemRandom

occupied = set()
for table in (Path("/proc/net/udp"), Path("/proc/net/udp6")):
    if table.exists():
        for line in table.read_text().splitlines()[1:]:
            occupied.add(int(line.split()[1].rsplit(":", 1)[1], 16))
for domain in SystemRandom().sample(range(40, 96), 56):
    first = 7400 + 250 * domain
    if not occupied.intersection(range(first, first + 250)):
        print(domain)
        break
else:
    raise SystemExit(1)
PY
); then
    fail 'No unused local DDS domain was found.' dds_domain_unavailable
fi
export ROS_DOMAIN_ID
export GZ_PARTITION="coptrans_${run_dir##*/}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
if [[ -d $root/../.coptrans-python/casadi ]]; then
    export PYTHONPATH="$root/../.coptrans-python${PYTHONPATH:+:$PYTHONPATH}"
fi
{
    printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nGZ_PARTITION=%s\n' \
        "$ROS_DISTRO" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$GZ_PARTITION"
    printf 'git_commit='
    git -C "$root" rev-parse HEAD
    printf 'gazebo_sim=%s\n' "$gazebo_version"
} >"$run_dir/environment.txt"
printf 'ROS_DOMAIN_ID=%s  GZ_PARTITION=%s\n' "$ROS_DOMAIN_ID" "$GZ_PARTITION"

if [[ ${config[3]} == False ]]; then
    command -v colcon >/dev/null || fail 'Colcon is required to build the workspace.' colcon_missing
    (
        cd -- "$workspace"
        CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" \
            colcon build --packages-up-to fwis_bringup --parallel-workers 2
    ) 2>&1 | tee "$run_dir/build.log"
fi
[[ -r $workspace/install/setup.bash ]] || fail 'Build the Colcon workspace before using --no-build.' overlay_missing
source "$workspace/install/setup.bash"
for package in fwis_bringup fwis_backends fwis_control ranger_mini_gz ros_gz_bridge rviz2; do
    ros2 pkg prefix "$package" >/dev/null || fail "ROS package $package is unavailable." ros_package_missing
done
if [[ ${config[4]} == on ]]; then
    ros2 pkg prefix rosbag2_transport >/dev/null || fail 'Recording requires rosbag2_transport.' rosbag_missing
fi

env --default-signal=INT,TERM setsid ros2 launch --noninteractive fwis_bringup test.launch.py \
    "run_config:=$run_dir/resolved_config.yaml" > >(tee "$run_dir/launch.log" 9>&-) 2>&1 9>&- &
launch_pid=$!
launch_code=0
wait "$launch_pid" || launch_code=$?
if [[ -n $interrupted ]]; then
    # Let Launch stop its own nodes before terminating any survivor in its session.
    for ((attempt=0; attempt<80; attempt++)); do
        kill -0 -- "-$launch_pid" 2>/dev/null || break
        sleep .1
    done
    stop_remaining
    wait "$launch_pid" 2>/dev/null || true
fi

python3 - "$run_dir" "$launch_code" "$interrupted" <<'PY'
from pathlib import Path
import sys
import yaml

folder, launch_code, signal = Path(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
path = folder / "result.yaml"
data = yaml.safe_load(path.read_text()) if path.exists() else {}
config = yaml.safe_load((folder / "resolved_config.yaml").read_text())
if data.get("status") == "passed" and config["record"] == "on" and not (folder / "bag" / "metadata.yaml").is_file():
    data.update(status="failed", reason="bag_recording_missing")
if signal:
    code = 130 if signal == "INT" else 143
    data.setdefault("status", "cancelled")
    if data["status"] == "cancelled":
        data["reason"] = "operator_interrupt"
    data["exit_signal"] = signal
elif data.get("status") == "passed" and launch_code == 0:
    code = 0
elif data.get("status") == "timed_out":
    code = 124
else:
    code = launch_code or 1
    data.setdefault("status", "launch_failed")
    data.setdefault("reason", "launch_exited_before_result")
data["exit_code"] = code
data["launch_exit_code"] = launch_code
temporary = path.with_suffix(".tmp")
temporary.write_text(yaml.safe_dump(data, sort_keys=False))
temporary.replace(path)
print(f"Result {data['status']} ({data.get('reason', '')})")
print(path)
sys.exit(code)
PY
