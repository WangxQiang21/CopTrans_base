#!/usr/bin/env bash
# Collect blind results, separate witness audits, and a serial cold-process latency series.
set -eo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
system_dir=$(realpath "${1:?qualified isolated system snapshot}")
batch=$(realpath -m "${2:?unused result directory}")
witness=$(realpath -m "${3:?unused witness directory}")
identity=$(realpath -m "${4:?unused identity directory}")
cohort=${5:?frozen holdout cohort index}
mkdir -p "$batch" "$witness" "$identity"
[[ ! -e $identity/binaries.sha256 ]]
source /opt/ros/humble/setup.bash
source "$system_dir/ros2_ws/install/setup.bash"
pkg=$system_dir/ros2_ws/install/fwis_hybrid_joint
commit=$(cat "$system_dir/source_commit")
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
    taskset -c "$cpu" python3 scripts/hybrid_v3/benchmark.py "$action" --cohort "$cohort" --split "$split" --profile "$profile" \
        --policy-source "$system_dir/ros2_ws/src/fwis_hybrid_joint/config/policy.json" --source-commit "$commit" --executable "$exe" --jobs "$jobs" --output "$file.jsonl" > "$file.log"
}
for split in core development holdout pruning; do
    for profile in standard baseline; do run witness "$split" "$profile" "$witness" 3 "${HYBRID_BENCH_CPUS:-3-6}"; done
done
for split in core development; do
    for profile in standard baseline; do run solve "$split" "$profile" "$batch" 4 "${HYBRID_BENCH_CPUS:-3-6}"; done
done
# Review general search on core/development before exposing blind holdout requests.
python3 - "$batch" <<'PY_CHECK'
import json,sys
from pathlib import Path
for split,total in (("core",72),("development",120)):
    for profile in ("baseline","standard"):
        records=list(map(json.loads,(Path(sys.argv[1])/f"solve_{split}_{profile}.jsonl").read_text().splitlines()))
        failures=[]
        for row in records:
            result=row["evidence"]
            if row["label"]=="invalid":
                ok=not result.get("plan")
            else:
                ok=(result.get("status")=="CERTIFIED_POLICY_COMPLIANT" and result.get("audit",{}).get("executable"))
                if row["label"]=="A":
                    ok=ok and result.get("metrics",{}).get("mode_sequence")=="A" and result["metrics"]["internal_stops"]==0
            if not ok: failures.append(row["case_id"])
        if len({r["case_id"] for r in records})!=total or failures:
            raise SystemExit(f"Holdout sealed: {split}/{profile} has {len(records)}/{total} records; failures {failures}")
PY_CHECK
if [[ -e "$batch/KEEP_HOLDOUT_SEALED" ]]; then
    echo "Development regression under investigation; blind holdout requests were not run." >&2
    exit 75
fi
for profile in standard baseline; do run solve holdout "$profile" "$batch" 4 "${HYBRID_BENCH_CPUS:-3-6}"; done
for profile in standard baseline; do run ablation pruning "$profile" "$batch" 3 "${HYBRID_BENCH_CPUS:-3-6}"; done
for profile in standard baseline; do run timing core "$profile" "$batch" 1 "${HYBRID_TIMING_CPU:-2}"; done
