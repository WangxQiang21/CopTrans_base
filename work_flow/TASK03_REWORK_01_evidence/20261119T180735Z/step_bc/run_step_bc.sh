#!/usr/bin/env bash
# TASK03 rework Step B/C evidence: targeted tests for the F-region rework.
set -u
REPO=/home/wq/whut_thesis/expv3_multi_virtual_a
OUT="$REPO/work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_bc"
cd "$REPO" || exit 1
source /opt/ros/noetic/setup.bash >/dev/null 2>&1
source catkin_ws/devel/setup.bash >/dev/null 2>&1
export PYTHONPATH="$REPO:$REPO/tests:$REPO/catkin_ws/src/fwis_v3_optimizer/src:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1

run() {
  local name="$1"; shift
  local start end rc t0 t1
  echo "=== $name :: $*" > "$OUT/$name.cmd"
  start=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  t0=$(date +%s.%N)
  timeout 1800 "$@" > "$OUT/$name.log" 2>&1
  rc=$?
  t1=$(date +%s.%N)
  end=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  printf 'exit=%d\nstart=%s\nend=%s\nwall_s=%s\n' \
    "$rc" "$start" "$end" "$(python3 -c "print($t1-$t0)")" > "$OUT/$name.meta"
  echo "--- $name exit=$rc wall=$(python3 -c "print(round($t1-$t0,2))")"
}

run formation_f_region_rework \
  python3 -m unittest tests.test_virtual_a_formation_corridor.FormationFRegionReworkTests
run f_only_historical_identity \
  python3 -m unittest tests.test_virtual_a_pipeline.G4PipelineTests.test_f_only_historical_model_and_context_hashes_are_unchanged
run playback_v2_binding_rebuild \
  python3 -m unittest tests.test_virtual_a_playback.UnionFormationEnvelopeTest.test_v2_template_payload_rebuilds_in_the_retained_binding
echo DONE