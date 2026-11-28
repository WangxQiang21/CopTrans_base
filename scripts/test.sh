#!/usr/bin/env bash
set -e -o pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --blade-bends ]]; then
    shift
    export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
    exec python3 "$root/scripts/blade_bends/run.py" "$@"
fi
if [[ ${1:-} == --blade-plan || ${1:-} == --blade-run ]]; then
    command=${1#--blade-}
    shift
    source /opt/ros/humble/setup.bash
    overlay="$root/ros2_ws/install"
    if [[ ! -r "$overlay/setup.bash" ||
          ! -d "$overlay/ranger_mini_description/share/ranger_mini_description/meshes/painted" ||
          ! -r "$overlay/ranger_mini_gz/share/ranger_mini_gz/config/ranger.gui.config" ]]; then
        (cd "$root/ros2_ws" && colcon build --packages-up-to ranger_mini_gz --cmake-args -DCMAKE_BUILD_TYPE=Release)
    fi
    source "$overlay/setup.bash"
    source_pkg="$root/ros2_ws/src/ranger_mini_gz"
    export PYTHONPATH="$source_pkg${PYTHONPATH:+:$PYTHONPATH}"
    exec python3 -m ranger_mini_gz.blade_experiment --archive "$root/reports/runs" \
        --scene-package-dir "$source_pkg" "$command" "$@"
fi
if [[ ${1:-} == --scene-only ]]; then
    shift
    scene=${1:?Choose a blade scene name}
    shift
    case "$scene" in
        blade_mountain_01|blade_right_angle_01|blade_roundabout_01|blade_mountain_hard|blade_right_angle_hard|blade_roundabout_hard) ;;
        *) echo "Unknown two-vehicle scene: $scene" >&2; exit 2 ;;
    esac
    source /opt/ros/humble/setup.bash
    overlay="$root/ros2_ws/install"
    mkdir -p "$root/reports/runs"
    exec 8>"$root/reports/runs/.$scene.lock"
    flock -n 8 || { echo "This blade scene is already in use." >&2; exit 2; }
    show_scene=true
    for arg in "$@"; do
        case "$arg" in --generate-only) show_scene=false ;; --gazebo) show_scene=true ;; esac
    done
    if [[ $show_scene == true ]]; then
        exec 9>"$root/reports/runs/.test.lock"
        flock -n 9 || { echo "A CopTrans simulation is already running." >&2; exit 2; }
    fi
    source_pkg="$root/ros2_ws/src/ranger_mini_gz"
    if [[ ! -r "$overlay/setup.bash" ||
          ! -d "$overlay/ranger_mini_description/share/ranger_mini_description/meshes/painted" ||
          ! -r "$overlay/ranger_mini_gz/share/ranger_mini_gz/config/ranger.gui.config" ]]; then
        (
            flock 7
            cd "$root/ros2_ws"
            colcon build --packages-up-to ranger_mini_gz --cmake-args -DCMAKE_BUILD_TYPE=Release
        ) 7>"$root/reports/runs/.blade-build.lock"
    fi
    source "$overlay/setup.bash"
    export PYTHONPATH="$source_pkg${PYTHONPATH:+:$PYTHONPATH}"
    exec python3 -m ranger_mini_gz.scene_generator "$scene" "$root/ros2_ws/generated/scenes/$scene" \
        --scene-package-dir "$source_pkg" --task blade_pair --gazebo "$@"
fi
if [[ ${1:-} == --planner-only ]]; then
    shift
    exec "$root/scripts/cts_plan_only.sh" "$@"
fi
workspace="$root/ros2_ws"
install_base=$(realpath -m -- "${COPTRANS_INSTALL_BASE:-$workspace/install}")
source_stamp="$install_base/.coptrans-source.sha256"
source_stamp_tmp=
temporary=$(mktemp "${TMPDIR:-/tmp}/coptrans-test.XXXXXXXX")
run_root=$(realpath -m -- "$root/reports/runs")
archive_tool="$workspace/src/fwis_bringup/fwis_bringup/experiment_log.py"
archive_dir=
spool_root=
run_dir=
launch_pid=
interrupted=

stop_remaining() {
    local attempt
    if [[ -z $launch_pid ]] || ! kill -0 -- "-$launch_pid" 2>/dev/null; then
        return 0
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
    trap - EXIT
    rm -f -- "$temporary"
    if [[ -n $source_stamp_tmp ]]; then
        rm -f -- "$source_stamp_tmp"
    fi
    stop_remaining
    if [[ -n $run_dir && ! -f $run_dir/result.yaml ]]; then
        if [[ -n $interrupted ]]; then
            printf 'status: cancelled\nreason: operator_interrupt\nexit_code: %s\n' "$code" >"$run_dir/result.yaml"
        else
            printf 'status: launch_failed\nreason: startup_or_launch_exited\nexit_code: %s\n' "$code" >"$run_dir/result.yaml"
        fi
    fi
    if [[ -n $run_dir ]]; then
        if python3 "$archive_tool" collect "$run_dir" "$archive_dir" --final; then
            rm -rf -- "$spool_root"
        else
            printf 'Archive incomplete. Working data retained at %s\n' "$run_dir" >&2
            [[ $code != 0 ]] || code=2
        fi
    elif [[ -n $spool_root ]]; then
        rmdir -- "$spool_root"
    fi
    exit "$code"
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

COPTRANS_RUN_ROOT="$run_root" COPTRANS_CONFIG_OUT="$temporary" \
    PYTHONPATH="$workspace/src/fwis_bringup:$workspace/src/ranger_mini_gz${PYTHONPATH:+:$PYTHONPATH}" \
    python3 "$workspace/src/fwis_bringup/fwis_bringup/test_config.py" "$@"
[[ -s $temporary ]] || exit 0

mapfile -t config < <(python3 - "$temporary" <<'PY'
import json, sys
with open(sys.argv[1]) as handle:
    config = json.load(handle)
for key in ("task", "scenario", "planner", "no_build", "record", "experiment", "gazebo_gui", "rviz_gui"):
    print(config[key])
PY
)

mkdir -p -- "$run_root"
exec 9>"$run_root/.test.lock"
if ! flock -n 9; then
    printf 'ERROR: another CopTrans test is already running.\n' >&2
    exit 2
fi
python3 "$archive_tool" reserve "$run_root"
spool_root=$(mktemp -d "${TMPDIR:-/tmp}/coptrans-run.XXXXXXXX")
archive_dir=$(mktemp -d "$run_root/$(date +%Y%m%d_%H%M%S)_XXXXXX")
run_dir="$spool_root/${archive_dir##*/}"
export COPTRANS_ARCHIVE_DIR="$archive_dir"
mkdir -p -- "$run_dir"/{metrics,trajectories,screenshots,ros}
{
    printf '%q' "$root/scripts/test.sh"
    if (($#)); then
        printf ' %q' "$@"
    fi
    printf '\n'
} >"$run_dir/command.txt"
cp -- "$temporary" "$run_dir/resolved_config.yaml"
python3 "$archive_tool" collect "$run_dir" "$archive_dir"
printf 'Experiment ID %s\n' "${archive_dir##*/}"
printf 'Experiment mode %s\n' "${config[5]}"
printf 'Run directory %s\n' "$archive_dir"

fail() {
    printf 'ERROR: %s\n' "$1" >&2
    printf 'status: launch_failed\nreason: %s\nexit_code: 2\n' "$2" >"$run_dir/result.yaml"
    exit 2
}

[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble setup is missing.' ros_setup_missing
if [[ ${config[6]} == True || ${config[7]} == True ]]; then
    [[ -n ${DISPLAY:-} || -n ${WAYLAND_DISPLAY:-} ]] || fail 'Gazebo and RViz2 require a graphical display.' display_missing
fi
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
export GZ_IP="${GZ_IP:-127.0.0.1}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
if [[ -d $root/../.coptrans-python/casadi ]]; then
    export PYTHONPATH="$root/../.coptrans-python${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ -z ${COPTRANS_CASADI_ROOT:-} ]]; then
    # The CasADi wheel also installs its native CMake SDK in the package directory.
    casadi_sdk=$(python3 - <<'PY'
from importlib.util import find_spec
from pathlib import Path

spec = find_spec("casadi")
locations = spec.submodule_search_locations if spec else ()
for folder in locations or ():
    if (Path(folder) / "cmake" / "casadi-config.cmake").is_file():
        print(folder)
        break
PY
)
    if [[ -n $casadi_sdk ]]; then export COPTRANS_CASADI_ROOT="$casadi_sdk"; fi
fi
{
    printf 'ROS_DISTRO=%s\nROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nGZ_PARTITION=%s\nGZ_IP=%s\n' \
        "$ROS_DISTRO" "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" "$GZ_PARTITION" "$GZ_IP"
    printf 'git_commit='
    git -C "$root" rev-parse HEAD
    git_state=$(git -C "$root" status --porcelain)
    if [[ -n $git_state ]]; then
        printf 'git_dirty=true\n'
    else
        printf 'git_dirty=false\n'
    fi
    printf 'gazebo_sim=%s\n' "$gazebo_version"
} >"$run_dir/environment.txt"
python3 "$archive_tool" collect "$run_dir" "$archive_dir"
python3 - "$run_dir" <<'PY'
import json
from pathlib import Path
import sys
run = Path(sys.argv[1])
config = json.loads((run / "resolved_config.yaml").read_text())
print(f"Configured planner {config['planner_backend']} | "
      f"tracker {config.get('tracking_implementation', 'python')}/{config['controller_algorithm']} | "
      f"profile {config.get('architecture_profile', 'baseline')} | "
      f"scene {config['scenario']}/{config.get('scene_variant', 'default')}")
if config.get("replay_of"):
    print(f"Replay of {config['replay_of']}")
    recorded = config.get("replay_git_commit")
    current = dict(line.split("=", 1) for line in (run / "environment.txt").read_text().splitlines())
    if recorded and recorded != current["git_commit"]:
        print(f"Replay uses the current code ({current['git_commit'][:8]}). Original used {recorded[:8]}.")
PY
printf 'ROS_DOMAIN_ID=%s  GZ_PARTITION=%s\n' "$ROS_DOMAIN_ID" "$GZ_PARTITION"

fingerprint() {
    python3 "$root/scripts/source_fingerprint.py" "$root"
}

source_digest=$(fingerprint) || fail 'Could not verify the CopTrans source fingerprint.' source_fingerprint_failed
if [[ ${config[3]} == False ]]; then
    command -v colcon >/dev/null || fail 'Colcon is required to build the workspace.' colcon_missing
    rm -f -- "$source_stamp"
    (
        cd -- "$workspace"
        CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" \
            colcon build --packages-up-to fwis_bringup --parallel-workers 2 --install-base "$install_base" \
                --cmake-args -DCMAKE_BUILD_TYPE=Release
    ) 2>&1 | tee "$run_dir/build.log"
    built_digest=$(fingerprint) || fail 'Could not verify the CopTrans source fingerprint after the build.' source_fingerprint_failed
    [[ $source_digest == "$built_digest" ]] || fail 'Source changed during the build. Rerun without --no-build.' source_changed_during_build
else
    if [[ ! -r $source_stamp ]] || ! read -r stamped_source stamped_install stamped_mode < "$source_stamp" || \
            [[ $stamped_source != "$source_digest" || $stamped_mode != Release || -z $stamped_install ]]; then
        fail 'The installed CopTrans build is stale or unverified. Rerun without --no-build. 请去掉 --no-build 重新运行。' overlay_source_mismatch
    fi
fi
[[ -r $install_base/setup.bash ]] || fail 'Build the Colcon workspace before using --no-build.' overlay_missing
source "$install_base/setup.bash"
if ! python3 - "$install_base" <<'PY'
from importlib.util import find_spec
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix, PackageNotFoundError

install = Path(sys.argv[1])
packages = ("fwis_backends", "fwis_bringup", "fwis_common", "fwis_control",
            "fwis_hybrid_joint", "fwis_hybrid_joint_ros", "fwis_multi_planner_cpp", "fwis_planning_core", "fwis_ros",
            "fwis_rviz_plugins", "fwis_v3_msgs", "ranger_mini_description", "ranger_mini_gz")
for package in packages:
    try:
        resolved = Path(get_package_prefix(package)).resolve()
    except PackageNotFoundError:
        sys.exit(f"Missing CopTrans overlay package: {package}")
    if resolved not in (install, install / package):
        sys.exit(f"{package} resolves to {resolved}, outside the selected install {install}")
for module in ("fwis_backends", "fwis_bringup", "fwis_control", "fwis_ros",
               "fwis_v3_msgs", "ranger_mini_gz", "fwis_multi_frontier_native",
               "fwis_planning_core_native"):
    spec = find_spec(module)
    if spec is not None and spec.origin and not Path(spec.origin).absolute().is_relative_to(install):
        sys.exit(f"{module} resolves to {spec.origin}, outside the selected install {install}")
PY
then
    fail 'CopTrans package paths do not match the selected install. Open a fresh shell and rerun without --no-build.' overlay_package_mismatch
fi
for package in ros_gz_bridge rviz2; do
    ros2 pkg prefix "$package" >/dev/null || fail "ROS package $package is unavailable." ros_package_missing
done
if [[ ${config[4]} == on ]]; then
    ros2 pkg prefix rosbag2_transport >/dev/null || fail 'Recording requires rosbag2_transport.' rosbag_missing
fi
install_receipt=$(python3 "$root/scripts/source_fingerprint.py" --installed-receipt "$install_base") || \
    fail 'Required CopTrans runtime files are missing. Rerun without --no-build.' overlay_artifact_missing
install_digest=${install_receipt%%$'\n'*}
installed_files=${install_receipt#*$'\n'}
if [[ ${config[3]} == False ]]; then
    source_stamp_tmp=$(mktemp "$install_base/.coptrans-source.XXXXXXXX")
    printf '%s %s Release\n' "$source_digest" "$install_digest" > "$source_stamp_tmp"
    mv -f -- "$source_stamp_tmp" "$source_stamp"
    source_stamp_tmp=
elif [[ $stamped_install != "$install_digest" ]]; then
    fail 'Installed CopTrans runtime files have changed. Rerun without --no-build. 请去掉 --no-build 重新运行。' overlay_artifact_mismatch
fi
printf 'coptrans_source_sha256=%s\ncoptrans_install_sha256=%s\ncoptrans_build_type=Release\nCOPTRANS_INSTALL_BASE=%s\n' \
    "$source_digest" "$install_digest" "$install_base" >> "$run_dir/environment.txt"
printf 'coptrans_installed_files=%s\n' "$installed_files" >> "$run_dir/environment.txt"
printf 'CopTrans Release install %s (source %s, runtime %s)\n' "$install_base" "${source_digest:0:12}" "${install_digest:0:12}"
python3 "$archive_tool" collect "$run_dir" "$archive_dir"

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

python3 - "$run_dir" "$archive_dir" "$launch_code" "$interrupted" <<'PY'
from pathlib import Path
import sys
import yaml

folder, archive, launch_code, signal = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
path = folder / "result.yaml"
data = yaml.safe_load(path.read_text()) if path.exists() else {}
config = yaml.safe_load((folder / "resolved_config.yaml").read_text())
if data.get("status") == "passed" and config["record"] == "on" and not (folder / "bag" / "metadata.yaml").is_file():
    data.update(status="failed", reason="bag_recording_missing")
if signal:
    code = 130 if signal == "INT" else 143
    if config.get("interactive") and data.get("status") == "passed" and signal == "INT":
        code = 0
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
if data.get("planner_source"):
    print("Observed planner source", data["planner_source"])
if data.get("model_sha256"):
    print("Received model SHA-256", data["model_sha256"])
print(archive / path.name)
sys.exit(code)
PY
