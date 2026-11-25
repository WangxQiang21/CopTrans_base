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
    "$bench" "$cars" "$seconds" "$output/$cars.csv" > "$output/$cars.summary.json"
done
