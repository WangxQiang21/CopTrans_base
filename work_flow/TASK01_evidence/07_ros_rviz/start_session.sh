#!/usr/bin/env bash
# Start one isolated demo session for Step 6.
# usage: start_session.sh <run_dir> <map_dir> <scale> <goal_policy>
set -u
cd "$(dirname "$0")/../../.."
run_dir=$1
map_dir=$2
scale=$3
policy=$4
echo "$$" > "$run_dir/session.pid"
mkdir -p "$run_dir/nodes_raw" "$run_dir/images"
# roslaunch starts node processes with cwd=$ROS_HOME (~/.ros), so a relative
# ROS_LOG_DIR would place the node logs outside the evidence directory.
export ROS_LOG_DIR="$(realpath "$run_dir/nodes_raw")"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export DISPLAY=:1
exec ./run_demo.sh --map "$map_dir" --scale "$scale" \
    backend:=virtual_rigid_a auto_goal:=false rviz:=true \
    virtual_a_goal_policy:="$policy" > "$run_dir/launch.log" 2>&1