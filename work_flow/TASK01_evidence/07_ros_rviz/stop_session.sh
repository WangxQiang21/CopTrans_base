#!/usr/bin/env bash
# Stop the roslaunch started by start_session.sh, leaving the private master up.
# usage: stop_session.sh <run_dir>
set -u
run_dir=$1
session_pid=$(cat "$run_dir/session.pid" 2>/dev/null || true)
roslaunch_pid=$(pgrep -P "${session_pid:-0}" -f roslaunch 2>/dev/null | head -1)
if [[ -n "${roslaunch_pid}" ]]; then
    kill -INT "$roslaunch_pid"
    for _ in $(seq 1 150); do
        kill -0 "$roslaunch_pid" 2>/dev/null || break
        sleep 0.2
    done
    if kill -0 "$roslaunch_pid" 2>/dev/null; then
        kill -TERM "$roslaunch_pid"
        sleep 1
    fi
fi
if [[ -n "${session_pid}" ]]; then
    for _ in $(seq 1 100); do
        kill -0 "$session_pid" 2>/dev/null || break
        sleep 0.1
    done
fi
echo "stopped roslaunch pid=${roslaunch_pid:-none} session pid=${session_pid:-none}" \
    > "$run_dir/stop.txt"