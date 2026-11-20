#!/usr/bin/env bash
# TASK03 rework mandatory verification commands (plan section 5).
# Runs from the rework worktree root; records exit code, wall time and
# stdout/stderr per command.  No command modifies frozen evidence.
set -u
REPO=/home/wq/whut_thesis/expv3_multi_virtual_a
OUT="$REPO/work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/mandatory"
cd "$REPO" || exit 1
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source catkin_ws/devel/setup.bash >/dev/null 2>&1
export PYTHONPATH="$REPO/tests:$REPO/catkin_ws/src/fwis_v3_optimizer/src:${PYTHONPATH:-}"

run() {
  local name="$1"; shift
  local start end rc
  echo "=== $name :: $*" > "$OUT/$name.cmd"
  start=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  local t0 t1
  t0=$(date +%s.%N)
  timeout 1800 "$@" > "$OUT/$name.log" 2>&1
  rc=$?
  t1=$(date +%s.%N)
  end=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  printf 'exit=%d\nstart=%s\nend=%s\nwall_s=%s\n' \
    "$rc" "$start" "$end" "$(python3 -c "print($t1-$t0)")" > "$OUT/$name.meta"
  echo "--- $name exit=$rc wall=$(python3 -c "print(round($t1-$t0,2))")"
}

run test_virtual_a_formation_corridor \
  python3 -m unittest test_virtual_a_formation_corridor
run test_virtual_a_corridor_optimizer \
  python3 -m unittest test_virtual_a_corridor_optimizer
run test_virtual_a_pipeline \
  python3 -m unittest test_virtual_a_pipeline
run test_virtual_a_audit_integration_bounds \
  python3 -m unittest test_virtual_a_audit test_virtual_a_integration_bounds
run test_virtual_a_collision \
  python3 -m unittest test_virtual_a_collision
run test_virtual_a_ros_adapter_playback_receiver \
  python3 -m unittest test_virtual_a_ros_adapter test_virtual_a_playback test_virtual_a_receiver
run test_formation_demo_assets \
  python3 -m unittest test_formation_demo_assets
run catkin_make \
  catkin_make -C catkin_ws -j4
run bash_n_run_demo \
  bash -n run_demo.sh
run run_demo_dry_run \
  ./run_demo.sh --map map05 --scale 2.625 --dry-run
echo DONE