#!/usr/bin/env bash
set -eo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
dir=$(realpath -m "${1:?new evidence directory}")
mkdir -p "$dir"
[[ ! -e $dir/bin ]]
source /opt/ros/humble/setup.bash
source "$root/ros2_ws/install/setup.bash"
cmake -S "$root/ros2_ws/src/fwis_hybrid_joint" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON > "$dir/configure.log"
cmake --build "$dir/build" --parallel 3 > "$dir/build.log"
ctest --test-dir "$dir/build" --output-on-failure --output-junit "$dir/ctest.xml"
mkdir "$dir/bin"
cp "$dir/build/hybrid_joint_case" "$dir/build/libfwis_hybrid_joint.so" "$dir/bin/"
# The executable's RUNPATH already refers to this immutable build directory.
ldd "$dir/bin/hybrid_joint_case" > "$dir/dependencies.txt"
sha256sum "$dir/bin/hybrid_joint_case" "$dir/build/libfwis_hybrid_joint.so" > "$dir/binaries.sha256"
(cd "$root" && git ls-files --cached --others --exclude-standard ros2_ws/src/fwis_hybrid_joint ros2_ws/src/fwis_planning_core | sort | xargs sha256sum) > "$dir/sources.sha256"
