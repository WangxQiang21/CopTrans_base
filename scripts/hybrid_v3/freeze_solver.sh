#!/usr/bin/env bash
# Compile without sourcing the existing workspace, whose shared library may have an older ABI.
set -eo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
dir=$(realpath -m "${1:?new evidence directory}")
mkdir -p "$dir"
[[ ! -e $dir/build ]]
source /opt/ros/humble/setup.bash
export COPTRANS_CASADI_ROOT=${COPTRANS_CASADI_ROOT:-$root/../.coptrans-python/casadi}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
cd "$root/ros2_ws"
colcon --log-base "$dir/log" build --packages-up-to fwis_hybrid_joint --build-base "$dir/build" \
    --install-base "$dir/install" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$dir/build.log" 2>&1
source "$dir/install/setup.bash"
for pkg in fwis_planning_core fwis_hybrid_joint; do
    ctest --test-dir "$dir/build/$pkg" --output-on-failure --output-junit "$dir/$pkg.xml"
done
mkdir "$dir/bin"
ln -s "$dir/build/fwis_hybrid_joint/hybrid_joint_case" "$dir/bin/hybrid_joint_case"
ldd "$dir/bin/hybrid_joint_case" > "$dir/dependencies.txt"
sha256sum "$dir/bin/hybrid_joint_case" "$dir/build/fwis_hybrid_joint/libfwis_hybrid_joint.so" > "$dir/binaries.sha256"
(cd "$root" && git ls-files --cached --others --exclude-standard ros2_ws/src/fwis_hybrid_joint ros2_ws/src/fwis_planning_core | sort | xargs sha256sum) > "$dir/sources.sha256"
