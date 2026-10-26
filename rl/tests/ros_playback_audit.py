#!/usr/bin/env python3
"""Drive the real playback node on a private ROS master and simulated clock.

Run with ROS Noetic and the overlay sourced. Never uses the user's ROS master.
Input is foundation/native_timing.json; output contains every observed marker.
"""
import argparse
import bisect
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"rl/src"))
from coptrans_rl.kernel_adapter import Pose
from coptrans_rl.timing_audit import classify_samples


def expected_pose(trace, t):
    times = [p[0] for p in trace]
    i = min(len(trace)-1, max(1, bisect.bisect_right(times, t)))
    a, b = trace[i-1], trace[i]
    ratio = max(0., min(1., (t-a[0])/(b[0]-a[0])))
    dyaw = math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3]))
    return a[1]+ratio*(b[1]-a[1]), a[2]+ratio*(b[2]-a[2]), a[3]+ratio*dyaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    os.environ.update(ROS_MASTER_URI="http://127.0.0.1:%d" % port,
                      ROS_HOSTNAME="localhost", ROS_IP="127.0.0.1",
                      ROS_HOME=str(args.output/"ros_home"), ROS_LOG_DIR=str(args.output/"ros_logs"))
    log = (args.output/"processes.log").open("w")
    processes = []
    try:
        processes.append(subprocess.Popen(["roscore", "-p", str(port)], stdout=log, stderr=log))
        master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
        deadline = time.monotonic()+15
        while True:
            try:
                if master.getPid("rl_playback_audit")[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("private_roscore_startup_timeout")
            time.sleep(.05)
        master.setParam("rl_playback_audit", "/use_sim_time", True)
        import rospy
        from geometry_msgs.msg import Pose2D
        from rosgraph_msgs.msg import Clock
        from visualization_msgs.msg import Marker
        from fwis_v3_msgs.msg import TimedTrajectory, TimedPoint
        rospy.init_node("rl_playback_audit", disable_signals=True)
        condition = threading.Condition()
        state = {"marker": None, "hud": "", "wire": None}

        def update(key, value):
            with condition:
                state[key] = value
                condition.notify_all()

        marker_sub = rospy.Subscriber("/robot/marker", Marker, lambda m: update("marker", m), queue_size=100)
        hud_sub = rospy.Subscriber("/robot/hud", Marker, lambda m: update("hud", m.text), queue_size=10)
        rospy.Subscriber("/plan/timed_trajectory", TimedTrajectory, lambda m: update("wire", m), queue_size=1)
        clock = rospy.Publisher("/clock", Clock, queue_size=10, latch=True)
        publish = rospy.Publisher("/plan/timed_trajectory", TimedTrajectory, queue_size=1, latch=True)
        processes.append(subprocess.Popen([sys.executable, str(ROOT/"catkin_ws/src/fwis_v3_playback/scripts/playback.py")],
                                          stdout=log, stderr=log))
        deadline = time.monotonic()+15
        while (publish.get_num_connections() < 2 or clock.get_num_connections() < 2 or
               marker_sub.get_num_connections() < 1 or hud_sub.get_num_connections() < 1):
            clock.publish(Clock(clock=rospy.Time.from_sec(1.)))
            if time.monotonic() > deadline:
                raise RuntimeError("playback_connection_timeout")
            time.sleep(.02)

        def wait_for(predicate, reason, timeout=3.):
            limit = time.monotonic()+timeout
            with condition:
                while not predicate():
                    remaining = limit-time.monotonic()
                    if remaining <= 0:
                        print("ROS wait diagnostics", reason, state["hud"],
                              state["marker"].header.stamp.to_sec() if state["marker"] else None,
                              flush=True)
                        raise RuntimeError(reason)
                    condition.wait(min(.05, remaining))

        report = {"scope": "real_ROS_serialization_and_playback_marker_not_full_planner",
                  "clock": "private_simulated_40ms", "cases": []}
        native = json.loads(args.input.read_text())
        epoch, plan_id = 2., 0
        for name, case in native["cases"].items():
            for method, stats in case["methods"].items():
                plan_id += 1
                trace = stats["timed_trace"]
                msg = TimedTrajectory(plan_id=plan_id)
                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.from_sec(epoch)
                msg.points = [TimedPoint(time_from_start=rospy.Duration.from_sec(t),
                                        center_pose=Pose2D(x=x, y=y, theta=th), v=v, mode=int(mode))
                              for t, x, y, th, v, mode in trace]
                with condition:
                    state.update(marker=None, hud="", wire=None)
                publish.publish(msg)
                clock.publish(Clock(clock=rospy.Time.from_sec(epoch)))
                wait_for(lambda: state["wire"] is not None and state["wire"].plan_id == plan_id,
                         "wire_trajectory_timeout")
                wire = state["wire"]
                time.sleep(.05)  # Let the independent playback subscriber receive this plan.
                wire_error = max(abs(point.time_from_start.to_sec()-row[0])
                                 for point, row in zip(wire.points, trace))
                observed, errors = [], []
                # Advance slightly beyond the timer period to avoid a 1 ns
                # float->ROS duration roundoff leaving the timer asleep.
                tick_dt = .04001
                steps = math.floor(stats["duration_s"]/tick_dt)
                for tick in range(1, steps+1):
                    current = epoch+tick_dt*tick
                    clock.publish(Clock(clock=rospy.Time.from_sec(current)))
                    wait_for(lambda: (state["marker"] is not None and
                                      state["marker"].header.stamp.to_sec() >= current-1e-7 and
                                      ("#%d " % plan_id) in state["hud"]), "marker_tick_timeout")
                    marker = state["marker"]
                    pose = marker.pose
                    yaw = 2*math.atan2(pose.orientation.z, pose.orientation.w)
                    relative = marker.header.stamp.to_sec()-epoch
                    expected = expected_pose(trace, relative)
                    errors.append([math.hypot(pose.position.x-expected[0], pose.position.y-expected[1]),
                                   abs(math.atan2(math.sin(yaw-expected[2]), math.cos(yaw-expected[2])))])
                    observed.append((relative, Pose(pose.position.x, pose.position.y, yaw), 0., 0))
                summary = classify_samples(observed)
                expected_holds = stats["expected_event_holds_s"]
                holds = summary["stationary_holds_s"]
                summary.update(case=name, method=method, wire_timestamp_max_error_s=wire_error,
                               xy_max_error_m=max(e[0] for e in errors),
                               yaw_max_error_rad=max(e[1] for e in errors),
                               playback_holds_match=len(holds) == len(expected_holds) and all(
                                   abs(a-b) <= .081 for a, b in zip(holds, expected_holds)),
                               expected_event_holds_s=expected_holds,
                               marker_trace=[[t, p.x, p.y, p.th] for t, p, _, _ in observed])
                report["cases"].append(summary)
                print(name, method, "holds", holds, "pose_error", summary["xy_max_error_m"], flush=True)
                epoch += stats["duration_s"]+1.
        report["passed"] = all(x["playback_holds_match"] and x["xy_max_error_m"] < 1e-6 and
                               x["yaw_max_error_rad"] < 1e-6 and x["wire_timestamp_max_error_s"] < 2e-9
                               for x in report["cases"])
        (args.output/"playback_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        return 0 if report["passed"] else 1
    finally:
        for process in reversed(processes):
            process.terminate()
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        log.close()


if __name__ == "__main__":
    raise SystemExit(main())
