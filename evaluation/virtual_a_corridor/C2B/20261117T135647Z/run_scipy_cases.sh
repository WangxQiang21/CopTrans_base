#!/usr/bin/env bash
# SLSQP-side corridor cases: prove the numeric SLSQP path carries the same
# corridor constraints and fail-closed behaviour as IPOPT.
set -u
cd "$(dirname "$0")"
ROOT=/home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONPATH="$ROOT/catkin_ws/src/fwis_v3_optimizer/src:$ROOT/tests:$ROOT"
: > scipy_exit_codes.log

run_case() {
  local scene=$1 out=$2
  shift 2
  local log="${out%.json}.stdout.log"
  echo "python3 -B run_pipeline_case.py $scene corridor $out --solver scipy $*" >> pipeline_commands.log
  python3 -B run_pipeline_case.py "$scene" "corridor" "$out" --solver scipy "$@" 2>&1 \
    | grep -v "RuntimeWarning\|warnings.warn" | tee "$log"
  local rc=${PIPESTATUS[0]}
  echo "$scene corridor/scipy $out EXIT=$rc" | tee -a scipy_exit_codes.log
  return $rc
}

run_case door_circumcircle door_circumcircle_corridor_scipy.json --max-seconds 40
run_case S1 S1_corridor_scipy.json --max-seconds 30
run_case S3 S3_corridor_scipy.json --max-seconds 40
run_case S2 S2_corridor_scipy.json --max-seconds 40
run_case S2_mirror S2_mirror_corridor_scipy.json --max-seconds 40