#!/usr/bin/env bash
# Convenience launcher for the G5 virtual_rigid_a three-vehicle backend.
#
# It is a thin wrapper around run_demo.sh: it only fills the parameters that
# the research entry needs when they are absent, and never copies map/GUI
# logic:
#   backend:=virtual_rigid_a
#   virtual_a_guidance_mode:=corridor
#   virtual_a_corridor_enforcement:=hard
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
has_guidance=false
has_enforcement=false
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
    virtual_a_guidance_mode:=*)
      has_guidance=true
      args+=("$arg")
      ;;
    virtual_a_corridor_enforcement:=*)
      has_enforcement=true
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
[[ "$has_guidance" == true ]] || args+=("virtual_a_guidance_mode:=corridor")
[[ "$has_enforcement" == true ]] || args+=("virtual_a_corridor_enforcement:=hard")
[[ "$has_auto_goal" == true ]] || args+=("auto_goal:=false")

exec "${root}/run_demo.sh" "${args[@]}"