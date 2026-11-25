#!/usr/bin/env bash
# Build one immutable Git snapshot; namespaces and clocks are measured on this exact overlay.
set -eo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
out=$(realpath -m "${1:?fresh local build path}")
sha=$(git -C "$root" rev-parse "${2:-HEAD}^{commit}")
mkdir -p "$out"
[[ ! -e $out/ros2_ws/src ]]
git -C "$root" archive "$sha" ros2_ws/src | tar -x -C "$out"
# Sparse clones can omit legacy archives under scripts. Extract only the tracked executable entry points.
for script in scripts/test.sh scripts/interactive.sh scripts/source_fingerprint.py; do
    mkdir -p "$out/scripts"
    git -C "$root" show "$sha:$script" > "$out/$script"
    chmod +x "$out/$script"
done
printf '%s\n' "$sha" > "$out/source_commit"
export COPTRANS_CASADI_ROOT=${COPTRANS_CASADI_ROOT:-$root/../.coptrans-python/casadi}
export CMAKE_BUILD_PARALLEL_LEVEL=3 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
source /opt/ros/humble/setup.bash
cd "$out/ros2_ws"
colcon build --packages-up-to fwis_bringup --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$out/build.log" 2>&1
source install/setup.bash
mkdir -p "$out/ctest"
test_code=0
for pkg in build/*; do
    if [[ -f $pkg/CTestTestfile.cmake ]]; then
        ctest --test-dir "$pkg" --output-junit "$out/ctest/${pkg##*/}.xml" --output-on-failure || test_code=1
    fi
done
python3 -m pytest -q --junitxml="$out/ctest/planner_pytest.xml" src/fwis_backends/test || test_code=1
exit "$test_code"
