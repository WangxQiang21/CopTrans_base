#!/usr/bin/env bash
# Run every C2-B pipeline evidence case sequentially, recording exact exit codes.
set -u
cd "$(dirname "$0")"
ROOT=/home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONPATH="$ROOT/catkin_ws/src/fwis_v3_optimizer/src:$ROOT/tests:$ROOT"
: > pipeline_exit_codes.log

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

run_case door_circumcircle legacy_route door_circumcircle_legacy_route.json --max-seconds 40
for scene in S1 S1_mirror S2 S2_mirror S3 S3_mirror; do
  run_case "$scene" corridor "${scene}_corridor.json"
done
for scene in S1 S1_mirror S2 S2_mirror S3 S3_mirror; do
  run_case "$scene" legacy_route "${scene}_legacy_route.json"
done