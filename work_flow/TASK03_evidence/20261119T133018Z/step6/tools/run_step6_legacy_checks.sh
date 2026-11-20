#!/usr/bin/env bash
# TASK03 Step 6 supplementary regression: the legacy/uniform transport path
# must still behave exactly as before the FormationPlaybackPlan.msg gained
# formation_binding_json and formation_playback.py learned to forward it.
set -u

ROOT="/home/wq/whut_thesis/expv3_multi_virtual_a"
OUT="$ROOT/work_flow/TASK03_evidence/20261119T133018Z/step6/tests"
mkdir -p "$OUT"

set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/catkin_ws/devel/setup.bash"
set -u

export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$ROOT:$ROOT/tests:$ROOT/catkin_ws/src/fwis_v3_optimizer/src:${PYTHONPATH:-}"
cd "$ROOT" || exit 1

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

run test_legacy_uniform_transport \
  python3 -m unittest test_uniform_playback_plan test_formation_playback \
    test_cargo_transport test_transport_contract

run test_legacy_uniform_regression \
  python3 -m unittest test_formation_regression test_uniform_pipeline

echo "legacy checks recorded under $OUT"