#!/usr/bin/env bash
# TASK03 Step 6 offline checks: every command records stdout/stderr, exit code,
# wall time and UTC start/end under step6/tests/.
set -u

ROOT="/home/wq/whut_thesis/expv3_multi_virtual_a"
OUT="$ROOT/work_flow/TASK03_evidence/20261119T133018Z/step6/tests"
mkdir -p "$OUT"

set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/catkin_ws/devel/setup.bash"
set -u

export PYTHONDONTWRITEBYTECODE=1
# The repository root is on PYTHONPATH because two test modules import their
# helpers as ``tests.test_virtual_a_*``; flat module names resolve through
# ``$ROOT/tests``.
export PYTHONPATH="$ROOT:$ROOT/tests:$ROOT/catkin_ws/src/fwis_v3_optimizer/src:${PYTHONPATH:-}"
cd "$ROOT" || exit 1

: > "$OUT/exit_codes.txt"
: > "$OUT/run_window.txt"

run() {
  local name="$1"
  shift
  local started ended code t0 t1
  started="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  t0="$(date +%s)"
  echo "----- $started $name: $*" >> "$OUT/run_window.txt"
  "$@" > "$OUT/$name.log" 2>&1
  code=$?
  t1="$(date +%s)"
  ended="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "----- $ended $name" >> "$OUT/run_window.txt"
  echo "$code" > "$OUT/$name.exit"
  echo "$name exit=$code wall_s=$((t1 - t0)) start=$started end=$ended" \
    | tee -a "$OUT/exit_codes.txt"
  echo "$started" > "$OUT/$name.start"
  echo "$ended" > "$OUT/$name.end"
  echo "$((t1 - t0))" > "$OUT/$name.wall_s.txt"
}

run test_virtual_a_pipeline \
  python3 -m unittest test_virtual_a_pipeline
run test_virtual_a_ros_adapter_playback \
  python3 -m unittest test_virtual_a_ros_adapter test_virtual_a_playback
run test_virtual_a_corridor_display_formation_hud \
  python3 -m unittest test_virtual_a_corridor_display test_formation_hud
run test_formation_demo_assets \
  python3 -m unittest test_formation_demo_assets
run test_virtual_a_receiver \
  python3 -m unittest test_virtual_a_receiver

modules="$(ls "$ROOT"/tests/test_virtual_a_*.py | xargs -n1 basename \
  | sed 's/\.py$//' | tr '\n' ' ')"
run test_virtual_a_regression_all \
  python3 -m unittest $modules

run catkin_make \
  catkin_make -C "$ROOT/catkin_ws" -j4
run bash_syntax \
  bash -n "$ROOT/run_demo.sh"
run run_demo_dry_run \
  "$ROOT/run_demo.sh" --map map05 --scale 2.625 --dry-run

echo "all checks recorded under $OUT"