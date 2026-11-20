#!/usr/bin/env bash
# TASK03 Step 6 one-run live driver: task-private master, isolated ROS_HOME,
# isolated ROS_LOG_DIR.  It starts roscore, starts one ./run_demo.sh session,
# runs ros_step6_capture.py, then stops only the process group and master it
# created.  It never touches another session's master.
set -u

usage() {
  echo "usage: run_step6_live.sh --name NAME --mode union|f_only|negative" \
       "--map MAP_DIR [--scale S] --reset X,Y,TH --goal X,Y,TH" \
       "[--planning-timeout S]" >&2
  exit 2
}

name=""
mode=""
map_dir=""
scale=""
reset=""
goal=""
planning_timeout="240"
while (($#)); do
  case "$1" in
    --name) name="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --map) map_dir="$2"; shift 2 ;;
    --scale) scale="$2"; shift 2 ;;
    --reset) reset="$2"; shift 2 ;;
    --goal) goal="$2"; shift 2 ;;
    --planning-timeout) planning_timeout="$2"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "unknown argument: $1" >&2; usage ;;
  esac
done
[[ -n "$name" && -n "$mode" && -n "$map_dir" && -n "$reset" && -n "$goal" ]] \
  || usage
case "$mode" in
  union) scope="three_vehicle_union_v1" ;;
  f_only) scope="f_only" ;;
  negative) scope="three_vehicle_union_v1" ;;
  *) echo "invalid mode: $mode" >&2; exit 2 ;;
esac

ROOT="/home/wq/whut_thesis/expv3_multi_virtual_a"
EVIDENCE="$ROOT/work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/live"
TOOLS="$EVIDENCE/tools"
RUN="$EVIDENCE/$name"
MASTER_PORT=11663
MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
mkdir -p "$RUN/ros_home" "$RUN/nodes_raw"
for entry in "$RUN"/*; do
  [[ -e "$entry" ]] && rm -rf -- "$entry"
done
mkdir -p "$RUN/ros_home" "$RUN/nodes_raw"

export ROS_MASTER_URI="$MASTER_URI"
export ROS_HOME="$RUN/ros_home"
export ROS_LOG_DIR="$RUN/nodes_raw"
export PYTHONDONTWRITEBYTECODE=1
export DISPLAY="${DISPLAY:-:1}"

set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/catkin_ws/devel/setup.bash"
set -u

started_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

{
  echo "TASK03 Step 6 live run: $name ($mode)"
  echo "UTC start: $started_utc"
  echo "Worktree: $ROOT"
  echo
  echo "Environment:"
  echo "  ROS_MASTER_URI=$ROS_MASTER_URI (task-private master)"
  echo "  ROS_HOME=$ROS_HOME"
  echo "  ROS_LOG_DIR=$ROS_LOG_DIR"
  echo "  DISPLAY=$DISPLAY"
  echo "  PYTHONDONTWRITEBYTECODE=1"
  echo
  echo "Launch command (verbatim):"
  echo "  ./run_demo.sh --map $map_dir${scale:+ --scale $scale} virtual_a_formation_corridor_scope:=$scope"
  echo
  echo "RViz-style inputs (same topics as a user):"
  echo "  reset $reset"
  echo "  goal  $goal"
  echo
} > "$RUN/command.txt"

roscore -p "$MASTER_PORT" > "$RUN/roscore.log" 2>&1 &
master_pid=$!
echo "$master_pid" > "$RUN/roscore.pid"
for _ in $(seq 1 100); do
  if timeout 5 rosnode list >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done
if ! timeout 5 rosnode list >/dev/null 2>&1; then
  echo "roscore did not become ready" >> "$RUN/command.txt"
  kill -INT "$master_pid" 2>/dev/null || true
  exit 1
fi

launch_args=("--map" "$map_dir")
[[ -n "$scale" ]] && launch_args+=("--scale" "$scale")
launch_args+=("virtual_a_formation_corridor_scope:=$scope")
setsid "$ROOT/run_demo.sh" "${launch_args[@]}" \
  > "$RUN/launch.log" 2>&1 &
session_pid=$!
echo "$session_pid" > "$RUN/session.pid"
for _ in $(seq 1 300); do
  pgid="$(ps -o pgid= -p "$session_pid" 2>/dev/null | tr -d ' ')"
  [[ -n "$pgid" ]] && break
  sleep 0.2
done
echo "${pgid:-$session_pid}" > "$RUN/session_pgid_actual.txt"

cd "$ROOT" || exit 1
python3 "$TOOLS/ros_step6_capture.py" \
  --run-dir "$RUN" --label "$name" --mode "$mode" \
  --reset "$reset" --goal "$goal" \
  --planning-timeout "$planning_timeout" \
  > "$RUN/capture_driver.log" 2>&1
driver_exit=$?
echo "driver exit code: $driver_exit (capture_driver.exit)" >> "$RUN/command.txt"
echo "$driver_exit" > "$RUN/capture_driver.exit"

timeout 15 rosparam dump "$RUN/params.yaml" >/dev/null 2>&1 \
  || echo "rosparam dump failed" > "$RUN/params_dump.stderr"
timeout 10 rostopic list -v > "$RUN/topics.txt" 2>&1 \
  || echo "rostopic list failed" > "$RUN/topics.stderr"
timeout 10 rosnode list > "$RUN/nodes.txt" 2>&1 \
  || echo "rosnode list failed" > "$RUN/nodes.stderr"

if [[ -n "${pgid:-}" ]]; then
  kill -INT -- "-$pgid" 2>/dev/null || true
fi
for _ in $(seq 1 100); do
  if ! kill -0 "$session_pid" 2>/dev/null; then
    break
  fi
  sleep 0.2
done
kill -TERM -- "-${pgid:-$session_pid}" 2>/dev/null || true
for _ in $(seq 1 50); do
  if ! kill -0 "$session_pid" 2>/dev/null; then
    break
  fi
  sleep 0.2
done
kill -KILL -- "-${pgid:-$session_pid}" 2>/dev/null || true
kill -INT "$master_pid" 2>/dev/null || true
for _ in $(seq 1 100); do
  if ! kill -0 "$master_pid" 2>/dev/null; then
    break
  fi
  sleep 0.2
done
kill -KILL "$master_pid" 2>/dev/null || true

{
  echo
  echo "Process control:"
  echo "  roscore -p $MASTER_PORT pid $master_pid"
  echo "  run_demo.sh session pid $session_pid pgid ${pgid:-unknown}"
  echo "  stop: kill -INT -- -${pgid:-unknown}; master stopped last"
  echo "  Other-session masters and roslaunch processes were never touched."
  echo "UTC end: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
} >> "$RUN/command.txt"

exit "$driver_exit"