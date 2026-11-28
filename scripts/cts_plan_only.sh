#!/usr/bin/env bash
set -eo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace="$root/ros2_ws"
install_base=$(realpath -m -- "${COPTRANS_INSTALL_BASE:-$workspace/install}")
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'EOF'
Usage: ./scripts/test.sh --planner-only [--suite historical|paper] [--case CTS_S3] [--dock-assignment auto|1,3,2]
       [--speed-profile standard|baseline] [--transport-only] [--no-build]

Run the frozen CTS cases with the native solver, without Gazebo or RViz.
EOF
    exit 0
fi
no_build=false
options=()
for argument in "$@"; do
    if [[ $argument == --no-build ]]; then
        no_build=true
    else
        options+=("$argument")
    fi
done
source /opt/ros/humble/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
if [[ -d $root/../.coptrans-python/casadi ]]; then
    export COPTRANS_CASADI_ROOT="${COPTRANS_CASADI_ROOT:-$root/../.coptrans-python/casadi}"
fi
source_digest=$(python3 "$root/scripts/source_fingerprint.py" "$root")
stamp="$install_base/.coptrans-source.sha256"
if ! read -r saved_source saved_installed build_mode <"$stamp" 2>/dev/null ||
        [[ $saved_source != "$source_digest" || $build_mode != Release ]] ||
        [[ $saved_installed != "$(python3 "$root/scripts/source_fingerprint.py" --installed "$install_base")" ]]; then
    if $no_build; then
        printf 'CopTrans install is stale. Run again without --no-build.\n' >&2
        exit 2
    fi
    mkdir -p "$root/reports/runs"
    exec 9>"$root/reports/runs/.test.lock"
    if ! flock -n 9; then
        printf 'The overlay is in use. Finish the running test before rebuilding.\n' >&2
        exit 2
    fi
    (
        cd "$workspace"
        CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}" colcon build \
            --packages-up-to fwis_bringup --parallel-workers 2 --install-base "$install_base" \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
    )
    [[ $source_digest == "$(python3 "$root/scripts/source_fingerprint.py" "$root")" ]]
    installed_digest=$(python3 "$root/scripts/source_fingerprint.py" --installed "$install_base")
    temp=$(mktemp "$stamp.XXXXXXXX")
    printf '%s %s Release\n' "$source_digest" "$installed_digest" >"$temp"
    mv -- "$temp" "$stamp"
    exec 9>&-
fi
source "$install_base/setup.bash"

run_root="$root/reports/runs"
mkdir -p -- "$run_root"
python3 -m fwis_bringup.experiment_log reserve "$run_root"
output=$(mktemp -d "$run_root/$(date +%Y%m%d_%H%M%S)_XXXXXX")
printf 'Offline experiment ID %s\n' "${output##*/}"
exec python3 -m fwis_bringup.cts_offline --root "$root" --output "$output" "${options[@]}"
