#!/usr/bin/env bash
# Collect blind results, separate witness audits, and a serial cold-process latency series.
set -eo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
system_dir=$(realpath "${1:?qualified isolated system snapshot}")
batch=$(realpath -m "${2:?unused result directory}")
witness=$(realpath -m "${3:?unused witness directory}")
identity=$(realpath -m "${4:?unused identity directory}")
mkdir -p "$batch" "$witness" "$identity"
[[ ! -e $identity/binaries.sha256 ]]
source /opt/ros/humble/setup.bash
source "$system_dir/ros2_ws/install/setup.bash"
pkg=$system_dir/ros2_ws/install/fwis_hybrid_joint
exe=$pkg/lib/fwis_hybrid_joint/hybrid_joint_case
lib=$pkg/lib/libfwis_hybrid_joint.so
sha256sum "$exe" "$lib" > "$identity/binaries.sha256"
ldd "$exe" > "$identity/dependencies.txt"
git ls-files ros2_ws/src/fwis_hybrid_joint ros2_ws/src/fwis_planning_core |
    (cd "$system_dir" && xargs sha256sum) > "$identity/sources.sha256"
mkdir -p "$batch/ctest"
cp "$system_dir"/ctest/*.xml "$system_dir/source_commit" "$batch/ctest/"
run() {
    local action=$1 split=$2 profile=$3 directory=$4 jobs=$5 cpu=$6
    local file=$directory/${action}_${split}_${profile}
    taskset -c "$cpu" python3 scripts/hybrid_v3/benchmark.py "$action" --cohort 1 --split "$split" --profile "$profile" \
        --executable "$exe" --jobs "$jobs" --output "$file.jsonl" > "$file.log"
}
for split in core development holdout pruning; do
    for profile in standard baseline; do run witness "$split" "$profile" "$witness" 3 "${HYBRID_BENCH_CPUS:-3-6}"; done
done
for split in core development holdout; do
    for profile in standard baseline; do run solve "$split" "$profile" "$batch" 4 "${HYBRID_BENCH_CPUS:-3-6}"; done
done
for profile in standard baseline; do run ablation pruning "$profile" "$batch" 3 "${HYBRID_BENCH_CPUS:-3-6}"; done
for profile in standard baseline; do run timing core "$profile" "$batch" 1 "${HYBRID_TIMING_CPU:-2}"; done
numeric=reports/acceptance/hybrid_v3_numeric
python3 scripts/hybrid_v3/necessary.py --executable "$exe" --output "$numeric/necessary_corridor_c3.json"
python3 scripts/hybrid_v3/necessary_aux.py --executable "$exe" --output "$numeric/necessary_aux_c3.json"
