#!/usr/bin/env bash
set -euo pipefail
# The two setups run serially on the host's steady clock. A native planner
# inside the benchmark process continuously submits fresh joint planning tasks.
root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
bench=${COPTRANS_NATIVE_LOAD_BINARY:-ros2_ws/build/fwis_control/hybrid_deadlines}
output=${1:?supply an unused local evidence directory}
seconds=${2:-605}
mkdir -p "$output"
exec 8>"$root/reports/acceptance/.hybrid-native-bench.lock"
flock -n 8
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
for cars in 1 3; do
    [[ ! -e $output/$cars.csv ]]
    sha256sum "$bench" > "$output/$cars.binary.sha256"
    ldd "$bench" > "$output/$cars.libraries.txt"
    # Apply the same physical-core partition as the simulation launch. The
    # planner inherits the background mask before the tracker threads start.
    PYTHONPATH="$root/ros2_ws/src/fwis_bringup${PYTHONPATH:+:$PYTHONPATH}" \
        python3 - "$bench" "$cars" "$seconds" "$output/$cars.csv" > "$output/$cars.summary.json" <<'PY'
import os
import sys
from fwis_bringup.control_affinity import allocate

bench, count, seconds, csv_path = sys.argv[1:]
lanes = allocate(range(1, int(count) + 1))
worker_cpus = [str(min(lane)) for lane in lanes.values()]
os.execv(bench, [bench, count, seconds, csv_path, *worker_cpus])
PY
done
