#!/usr/bin/env bash
# Isolated local DDS tests for the installed planner, receiver, permit and C++ trackers.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
out=${1:-"$root/reports/acceptance/hybrid_v3/ros_qualification"}
domain=${ROS_DOMAIN_ID:?Set ROS_DOMAIN_ID to an unused test domain}
if [[ ! $domain =~ ^[0-9]+$ || $domain -lt 1 || $domain -gt 230 ]]; then
    echo 'ROS_DOMAIN_ID must be an isolated domain in the range 1-230.' >&2
    exit 2
fi
mkdir -p "$out"
# Humble setup references unset shell variables.
set +u
source /opt/ros/humble/setup.bash
source "${COPTRANS_INSTALL_BASE:-$root/ros2_ws/install}/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=$domain OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR="$out/ros"
fixture="$root/ros2_ws/src/fwis_hybrid_joint_ros/test/ros_group_faults.py"
hashes(){
    local lib planner control
    lib=$(ros2 pkg prefix fwis_hybrid_joint)
    planner=$(ros2 pkg prefix fwis_hybrid_joint_ros)
    control=$(ros2 pkg prefix fwis_control)
    sha256sum "$lib/lib/libfwis_hybrid_joint.so" "$planner/lib/libhybrid_codec.so" \
        "$planner/lib/fwis_hybrid_joint_ros/hybrid_joint_planner" "$planner/lib/fwis_hybrid_joint_ros/hybrid_joint_executor" \
        "$control/lib/fwis_control/ranger_tracking_controller_cpp"
}
hashes >"$out/binaries.before.sha256"
git -C "$root" rev-parse HEAD >"$out/source_commit"
failed=0
for n in 1 3; do
    for profile in standard baseline; do
        for trial in faults expired rollback repeat_ack ack_before_envelope latency; do
            case "$trial" in
                faults) args=(--cases success tamper missing_member stale_feedback wrong_mode map_replaced permit_loss wrong_epoch future_permit revoked_permit nan_feedback);;
                repeat_ack|ack_before_envelope) args=(--cases "$trial");;
                rollback) args=(--cases clock_rollback);;
                expired) args=(--cases expired);;
                latency) args=(--admission-trials 30);;
            esac
            name="${n}_${profile}_${trial}"
            python3 "$fixture" --vehicles "$n" --speed-profile "$profile" "${args[@]}" \
                --out "$out/$name.json" >"$out/$name.log" 2>&1 || failed=1
        done
    done
done
hashes >"$out/binaries.after.sha256"
cmp -s "$out/binaries.before.sha256" "$out/binaries.after.sha256" || failed=1
exit "$failed"
