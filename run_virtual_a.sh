#!/usr/bin/env bash
# Convenience launcher for the G5 virtual_rigid_a three-vehicle backend.
#
# It deliberately does NOT change run_demo.sh's accepted default backend
# (uniform_v02).  It only injects the two parameters virtual_rigid_a needs:
#   backend:=virtual_rigid_a
#   auto_goal:=false   (a manual 2D Pose Estimate is mandatory; auto_goal
#                       would publish a goal before the reset is confirmed)
#
# Usage:
#   ./run_virtual_a.sh --map map01 --scale 0.75
#   ./run_virtual_a.sh --map /path/to/map --scale 1.0 virtual_a_goal_policy:=straight
#   ./run_virtual_a.sh --map map06 --scale 1.0 --dry-run
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

has_backend=false
has_auto_goal=false
args=()
for arg in "$@"; do
  case "$arg" in
    backend:=*)
      if [[ "$arg" != "backend:=virtual_rigid_a" ]]; then
        echo "错误：run_virtual_a.sh 固定使用 backend:=virtual_rigid_a；" \
             "如需其他后端请直接运行 ./run_demo.sh。" >&2
        exit 2
      fi
      has_backend=true
      args+=("$arg")
      ;;
    auto_goal:=*)
      has_auto_goal=true
      args+=("$arg")
      ;;
    *)
      args+=("$arg")
      ;;
  esac
done

if [[ "$has_backend" == false ]]; then
  args=("backend:=virtual_rigid_a" "${args[@]}")
fi
if [[ "$has_auto_goal" == false ]]; then
  args+=("auto_goal:=false")
fi

exec "${root}/run_demo.sh" "${args[@]}"