#!/usr/bin/env bash
# Step 5 §5.1 authoritative quick-test groups, one process per group.
# Each group writes stdout/stderr/exit code/duration separately; the total
# exit code of a unittest discover run never masks an individual failure.
set -u
cd "$(dirname "$0")/../../.."
OUT=work_flow/TASK01_evidence/05_numerics/tests
mkdir -p "$OUT"
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:${PYTHONPATH:-}
: > "$OUT/summary.txt"

run_group () {
    name=$1
    shift
    started=$(date +%s.%N)
    python3 -m unittest "$@" > "$OUT/$name.stdout" 2> "$OUT/$name.stderr"
    code=$?
    finished=$(date +%s.%N)
    duration=$(python3 -c "print('{:.2f}'.format($finished - $started))")
    tail -3 "$OUT/$name.stderr" | tr '\n' ' ' > "$OUT/$name.result"
    printf '%s exit=%s elapsed=%ss command=%s\n' \
        "$name" "$code" "$duration" "$*" >> "$OUT/summary.txt"
    printf '%s exit=%s %s\n' "$name" "$code" "$(cat "$OUT/$name.result")"
}

run_group g1 tests.test_virtual_a_corridor_display tests.test_formation_hud
run_group g2 tests.test_virtual_a_receiver tests.test_virtual_a_playback
run_group g3 tests.test_formation_demo_assets
run_group g4 tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer
run_group g5 tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery
run_group g6 tests.test_virtual_a_geometry tests.test_virtual_a_profile
run_group g7 tests.test_virtual_a_integration_bounds \
    tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit \
    tests.test_virtual_a_trajectory