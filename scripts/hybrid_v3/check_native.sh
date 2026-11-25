#!/usr/bin/env bash
# Rebuild both native libraries from source, with no pybind target. Raw results
# and all instrumented objects stay in the local, ignored acceptance directory.
set -eo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
variant=${1:-Release}
out=${2:-"$root/reports/acceptance/hybrid_v3_numeric/$variant"}
case "$variant" in
 Release) flags="-O2 -g" ;;
 ASan_UBSan) flags="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" ;;
 TSan) flags="-O1 -g -fsanitize=thread -fno-omit-frame-pointer" ;;
 *) echo "Supported variants: Release, ASan_UBSan, TSan" >&2; exit 2 ;;
esac
mkdir -p "$out"
out=$(realpath "$out")
source /opt/ros/humble/setup.bash
# Prefer package discovery in an installed C++ SDK. The project's local SDK
# distribution has no Python imports and works with pybind disabled.
export COPTRANS_CASADI_ROOT=${COPTRANS_CASADI_ROOT:-"$root/../.coptrans-python/casadi"}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
export ASAN_OPTIONS=halt_on_error=1:detect_leaks=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
export TSAN_OPTIONS=halt_on_error=1
common=(-DCMAKE_BUILD_TYPE=RelWithDebInfo "-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=$flags" -DBUILD_TESTING=ON)
identifier=$(python3 "$root/scripts/hybrid_v3/numeric_manifest.py" "$out" --freeze)
src="$out/sources/$identifier/ros2_ws/src"
build="$out/build/$identifier"
cmake -S "$src/fwis_planning_core" -B "$build/core" "${common[@]}" -DFWIS_PLANNING_CORE_PYTHON=OFF "-DCMAKE_INSTALL_PREFIX=$build/install"
cmake --build "$build/core" -j2 --target install
cmake -S "$src/fwis_hybrid_joint" -B "$build/joint" "${common[@]}" "-Dfwis_planning_core_DIR=$build/install/share/fwis_planning_core/cmake"
cmake --build "$build/joint" -j2
( cd "$build/core" && ctest --output-on-failure --output-junit "$out/core.xml" )
( cd "$build/joint" && ctest --output-on-failure --output-junit "$out/joint.xml" )
"$build/joint/hybrid_model_oracle" "$out/model.csv" > "$out/model.json"
"$build/joint/hybrid_collision_oracle" "$out/collision.csv" > "$out/collision.json"
"$build/joint/hybrid_derivative_oracle" > "$out/derivatives.json"
python3 "$root/scripts/hybrid_v3/numeric_manifest.py" "$out"
