#!/usr/bin/env bash
set -eo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -z ${COPTRANS_EVIDENCE_DIR:-} ]]; then
    exec python3 "$root/scripts/record_acceptance.py" \
        --output "$root/reports/acceptance/$(date +%Y%m%d_%H%M%S)_ci_$$" \
        --input "$root/ros2_ws/src/fwis_bringup/config/test_defaults.yaml" \
        --input "$root/ros2_ws/src/fwis_control/config/ranger_tracking.yaml" \
        -- bash "$root/scripts/check.sh"
fi

source /opt/ros/humble/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
evidence=$COPTRANS_EVIDENCE_DIR
workspace=$(mktemp -d "${TMPDIR:-/tmp}/coptrans-check.XXXXXXXX")
finish() {
    local code=$?
    if [[ -d $workspace/build ]]; then
        mkdir -p "$evidence/colcon-results"
        (
            cd "$workspace/build"
            find . -path '*/test_results/*' -type f -name '*.xml' \
                -exec cp --parents {} "$evidence/colcon-results" \;
        )
    fi
    rm -rf -- "$workspace"
    exit "$code"
}
trap finish EXIT
cd "$root/ros2_ws"
colcon --log-base "$workspace/log" build --packages-up-to fwis_bringup \
    --build-base "$workspace/build" --install-base "$workspace/install" \
    --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source "$workspace/install/setup.bash"
python3 - <<'PY'
import fwis_multi_frontier_native
import fwis_planning_core_native
from fwis_backends.multi_4wis import _formation_collision_native

for module in (fwis_multi_frontier_native, fwis_planning_core_native, _formation_collision_native):
    print(f"{module.__name__}: {module.__file__}", flush=True)
PY
mkdir -p "$evidence/ctest"
test_status=0
for package_build in "$workspace/build"/*; do
    if [[ -f $package_build/CTestTestfile.cmake ]]; then
        package_name=${package_build##*/}
        ctest --test-dir "$package_build" --build-config Release --output-on-failure \
            --output-junit "$evidence/ctest/$package_name.xml" || test_status=1
    fi
done
python3 -m pytest -q --junitxml="$evidence/backend-tests.xml" src/fwis_backends/test
python3 -m pytest -q --junitxml="$evidence/entry-and-qualification-tests.xml" \
    "$root/scripts/test_public_entry.py" "$root/scripts/hybrid_v3"
exit "$test_status"
