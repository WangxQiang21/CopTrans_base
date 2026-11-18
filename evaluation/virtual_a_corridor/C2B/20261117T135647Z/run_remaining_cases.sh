#!/usr/bin/env bash
# Continuation after the first run_all_cases.sh process was killed at
# S1_mirror: re-run the interrupted case and every not-yet-run case.
# Completed before the kill: door_circumcircle corridor/legacy_route, S1 corridor.
set -u
cd "$(dirname "$0")"
ROOT=/home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONPATH="$ROOT/catkin_ws/src/fwis_v3_optimizer/src:$ROOT/tests:$ROOT"

run_case() {
  local scene=$1 mode=$2 out=$3
  shift 3
  local log="${out%.json}.stdout.log"
  echo "python3 -B run_pipeline_case.py $scene $mode $out $*" >> pipeline_commands.log
  python3 -B run_pipeline_case.py "$scene" "$mode" "$out" "$@" 2>&1 \
    | tee "$log"
  local rc=${PIPESTATUS[0]}
  echo "$scene $mode $out EXIT=$rc" | tee -a pipeline_exit_codes.log
  return $rc
}

run_case S1_mirror corridor S1_mirror_corridor.json
for scene in S2 S2_mirror S3 S3_mirror; do
  run_case "$scene" corridor "${scene}_corridor.json"
done
for scene in S1 S1_mirror S2 S2_mirror S3 S3_mirror; do
  run_case "$scene" legacy_route "${scene}_legacy_route.json"
done