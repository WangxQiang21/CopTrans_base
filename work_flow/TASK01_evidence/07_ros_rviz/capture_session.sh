#!/usr/bin/env bash
# Capture the §5.3 parameter/topic/RViz evidence of one live session.
# usage: capture_session.sh <run_dir> [tag]
set -u
cd "$(dirname "$0")/../../.."
run_dir=$1
tag=${2:-launch}
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11633}"
export DISPLAY="${DISPLAY:-:1}"
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
mkdir -p "$run_dir/images"

{
    echo "backend:"
    rosparam get /formation_uniform_planner/backend || true
    echo "guidance:"
    rosparam get /formation_uniform_planner/virtual_a_guidance_mode || true
    echo "enforcement:"
    rosparam get /formation_uniform_planner/virtual_a_corridor_enforcement || true
    echo "goal_policy:"
    rosparam get /formation_uniform_planner/virtual_a_goal_policy || true
} > "$run_dir/params_${tag}.txt" 2>&1

rosnode list > "$run_dir/nodes_${tag}.txt" 2>&1
rostopic list > "$run_dir/topics_${tag}.txt" 2>&1
: > "$run_dir/topic_samples_${tag}.txt"
for topic in /formation/planning_debug /formation/job_state \
             /formation/quality_report /formation/playback_plan; do
    name=$(echo "${topic#/formation/}" | tr '/' '_')
    timeout 5 rostopic echo -n1 "$topic" \
        > "$run_dir/sample_${name}_${tag}.txt" \
        2> "$run_dir/sample_${name}_${tag}.stderr"
    echo "$topic exit=$? bytes=$(wc -c < "$run_dir/sample_${name}_${tag}.txt")" \
        >> "$run_dir/topic_samples_${tag}.txt"
done

python3 - "$run_dir/images/${tag}.png" <<'PY'
import sys
sys.path.insert(0, "tests")
import ros_virtual_a_smoke as smoke
result = smoke.screenshot(__import__("pathlib").Path(sys.argv[1]))
print(result)
PY