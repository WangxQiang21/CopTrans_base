#!/usr/bin/env bash
# Post-fix §5.3 dry-run matrix (run after the formal catkin_make).
set -u
cd "$(dirname "$0")/../../../.."
out=work_flow/TASK01_evidence/06_regression/postfix/dry_runs
mkdir -p "$out"
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash

run() {
    name=$1
    shift
    /usr/bin/time -f "elapsed=%e s" -o "$out/$name.time" \
        ./run_demo.sh "$@" > "$out/$name.stdout" 2> "$out/$name.stderr"
    echo "$?" > "$out/$name.exit_code"
    echo "$name exit=$(cat "$out/$name.exit_code") $(cat "$out/$name.time")"
}

run dry_run_map05_scale2625 --map map05 --scale 2.625 --dry-run
run dry_run_map05_default --map map05 --dry-run
run dry_run_uniform_v02 --backend uniform_v02 --dry-run
run dry_run_legacy --backend legacy --dry-run
run dry_run_single --single --map map05 --dry-run
run dry_run_override --map map05 --scale 2.625 --dry-run \
    virtual_a_guidance_mode:=legacy_route \
    virtual_a_corridor_enforcement:=soft