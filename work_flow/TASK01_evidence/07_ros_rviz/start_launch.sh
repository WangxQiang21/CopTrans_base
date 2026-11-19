#!/usr/bin/env bash
# Start one literal run_demo.sh smoke on the task-private master.
# usage: start_launch.sh <run_dir> <run_demo arguments...>
# The caller supplies ROS_MASTER_URI; node logs land inside <run_dir>.
set -u
cd "$(dirname "$0")/../../.."
run_dir=$1
shift
echo "$$" > "$run_dir/session.pid"
mkdir -p "$run_dir/nodes_raw" "$run_dir/images"
export ROS_LOG_DIR="$(realpath "$run_dir/nodes_raw")"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export DISPLAY=:1
exec ./run_demo.sh "$@" > "$run_dir/launch.log" 2>&1