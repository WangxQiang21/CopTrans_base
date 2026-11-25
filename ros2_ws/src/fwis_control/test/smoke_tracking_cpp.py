#!/usr/bin/env python3
"""Run installed ROS 2 trackers against a synthetic vehicle plant."""
import argparse
import json
import math
from pathlib import Path
import subprocess
from time import monotonic

from ament_index_python.packages import get_package_prefix
from fwis_v3_msgs.msg import FormationExecutionPermit, TimedTrajectory
from geometry_msgs.msg import Twist as RosTwist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Int32, String

from fwis_control.model import Pose, Twist, ZERO, advance
from ros_fixtures import trajectory_message
from scenarios import A, O, S, Leg, sample_case


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


class Harness(Node):
    def __init__(self, count, algorithm, implementation):
        super().__init__("native_tracking_probe")
        self.ids = tuple(range(1, count + 1))
        self.pose = dict.fromkeys(self.ids, Pose(0., 0., 0.))
        self.command = dict.fromkeys(self.ids, ZERO)
        self.mode = dict.fromkeys(self.ids, 0)
        self.commands = dict((i, []) for i in self.ids)
        self.rows = dict((i, []) for i in self.ids)
        self.outputs = {}
        self.omit = set()
        self.permit_valid = True
        self.permit_key = None
        self.procs = []
        self.job_start = 0.
        node = {"python": "ranger_tracking_controller", "cpp": "ranger_tracking_controller_cpp"}
        executable = (Path(get_package_prefix("fwis_control")) / "lib" / "fwis_control" /
                      node[implementation])
        self.permit = self.create_publisher(FormationExecutionPermit, "/formation/execution_permit", 1)
        for i in self.ids:
            prefix = f"/robot{i}"
            self.procs.append(subprocess.Popen([
                str(executable), "--ros-args", "-r", f"__ns:={prefix}",
                "-p", f"trajectory_topic:={prefix}/trajectory", "-p", f"algorithm:={algorithm}",
                "-p", "permit_topic:=/formation/execution_permit", "-p", "future_start_window_s:=2.0"]))
            self.outputs[i] = (
                self.create_publisher(Odometry, f"{prefix}/odom", 5),
                self.create_publisher(Int32, f"{prefix}/motion_state", 5),
                self.create_publisher(TimedTrajectory, f"{prefix}/trajectory",
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            self.create_subscription(RosTwist, f"{prefix}/cmd_vel",
                                     lambda msg, i=i: self.on_command(i, msg), 10)
            self.create_subscription(Int32, f"{prefix}/sim_request_mode",
                                     lambda msg, i=i: self.on_request(i, msg.data), 5)
            self.create_subscription(String, f"{prefix}/controller_status",
                                     lambda msg, i=i: self.rows[i].append(json.loads(msg.data)), 10)
        self.last = monotonic()
        self.create_timer(.02, self.update)

    def on_command(self, index, msg):
        cmd = Twist(msg.linear.x, msg.linear.y, msg.angular.z)
        self.command[index] = cmd
        self.commands[index].append(cmd.values())

    def on_request(self, index, value):
        if value >= 0:
            self.mode[index] = value

    def update(self):
        wall = monotonic()
        dt, self.last = min(wall - self.last, .06), wall
        stamp = self.get_clock().now().to_msg()
        for i, (odometry, mode, _) in self.outputs.items():
            cmd = self.command[i]
            self.pose[i] = pose = advance(self.pose[i], cmd, dt)
            if i not in self.omit:
                msg = Odometry()
                msg.header.stamp = stamp
                msg.header.frame_id = "map"
                msg.child_frame_id = f"robot{i}/base_link"
                msg.pose.pose.position.x, msg.pose.pose.position.y = pose.x, pose.y
                msg.pose.pose.orientation.z = math.sin(pose.yaw * .5)
                msg.pose.pose.orientation.w = math.cos(pose.yaw * .5)
                msg.twist.twist.linear.x, msg.twist.twist.linear.y = cmd.vx, cmd.vy
                msg.twist.twist.angular.z = cmd.wz
                odometry.publish(msg)
            mode.publish(Int32(data=self.mode[i]))
        if self.permit_key is not None:
            plan_id, start = self.permit_key
            self.permit.publish(FormationExecutionPermit(
                stamp=stamp, plan_id=plan_id, start=start, enabled=self.permit_valid))

    def until(self, predicate, timeout):
        deadline = monotonic() + timeout
        while not predicate() and monotonic() < deadline:
            assert all(process.poll() is None for process in self.procs)
            rclpy.spin_once(self, timeout_sec=.02)
        return predicate()

    def prepare(self, scenario, plan_id):
        self.omit.clear()
        self.permit_valid = True
        self.pose = dict.fromkeys(self.ids, Pose(0., 0., 0.))
        self.mode = dict.fromkeys(self.ids, 0)
        assert self.until(lambda: self.permit.get_subscription_count() == len(self.ids) and
                          all(output[-1].get_subscription_count() and len(self.commands[i]) >= 3 and
                              self.count_publishers(f"/robot{i}/controller_status")
                              for i, output in self.outputs.items()), 10.)
        start = Time(nanoseconds=self.get_clock().now().nanoseconds + 1_000_000_000).to_msg()
        self.permit_key = plan_id, start
        self.job_start = monotonic()
        msg = trajectory_message(scenario.trajectory, start)
        msg.plan_id = plan_id
        for _, _, publisher in self.outputs.values():
            publisher.publish(msg)
        assert self.until(lambda: all(self.admitted(i, plan_id) for i in self.ids), 2.), {
            i: [r for r in rows if r.get("kind") == "admission"] for i, rows in self.rows.items()}
        return msg

    def admitted(self, i, plan_id):
        return any(row.get("plan_id") == plan_id and row.get("result") in ("PASS", "WARN")
                   for row in self.rows[i])

    def phase(self, i, plan_id, phase):
        return next((row for row in self.rows[i] if row.get("plan_id") == plan_id and
                     row.get("kind") == "execution" and row.get("phase") == phase), None)

    def teardown(self):
        for proc in self.procs:
            if proc.poll() is None:
                proc.terminate()
        for proc in self.procs:
            try:
                proc.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        self.destroy_node()


def exercise(harness):
    mixed = sample_case("native_mixed", [Leg(A, .24, .14, parameter=.7),
                        Leg(O, .22, .17, parameter=.8), Leg(S, .44, .27), Leg(A, .22, .15)])
    straight = sample_case("native_straight", [Leg(A, .9, .14)])
    harness.prepare(mixed, 1)
    assert harness.until(lambda: all(harness.phase(i, 1, "COMPLETE") for i in harness.ids), 20.)
    for i, rows in harness.rows.items():
        modes = {row.get("requested_mode") for row in rows if row.get("plan_id") == 1}
        assert {0, 1, 2} <= modes, (i, modes)
        goal = mixed.trajectory.points[-1].pose
        actual = harness.pose[i]
        assert math.hypot(actual.x - goal.x, actual.y - goal.y) < .05
    harness.prepare(straight, 2)
    assert harness.until(lambda: all(harness.command[i].speed > .03 for i in harness.ids), 4.)
    victim = harness.ids[-1]
    harness.omit.add(victim)
    assert harness.until(lambda: harness.phase(victim, 2, "STOPPED"), 1.)
    assert harness.phase(victim, 2, "STOPPED")["reasons"] == ["missing_stale_or_invalid_feedback"]
    harness.permit_valid = False
    assert harness.until(lambda: all(harness.phase(i, 2, "STOPPED") for i in harness.ids), 1.)
    assert harness.until(lambda: all(harness.command[i] == ZERO for i in harness.ids), 1.)
    for i in harness.ids[:-1]:
        assert harness.phase(i, 2, "STOPPED")["reasons"] == ["formation_permit_missing_or_revoked"]

    msg = harness.prepare(straight, 3)
    assert harness.until(lambda: all(harness.command[i].speed > .03 for i in harness.ids), 4.)
    victim = harness.ids[0]
    msg.plan_id += 1
    msg.header.stamp = harness.get_clock().now().to_msg()
    harness.outputs[victim][-1].publish(msg)
    assert harness.until(lambda: harness.phase(victim, 3, "STOPPED"), 1.)
    assert harness.phase(victim, 3, "STOPPED")["reasons"] == ["trajectory_replaced_during_execution"]
    harness.permit_valid = False
    assert harness.until(lambda: all(harness.phase(i, 3, "STOPPED") for i in harness.ids), 1.)
    assert harness.until(lambda: all(harness.command[i] == ZERO for i in harness.ids), 1.)


def stress(harness, seconds):
    straight = sample_case("native_pressure", [Leg(A, (seconds + 5.) * .14, .14)])
    harness.prepare(straight, 5)
    assert harness.until(lambda: False, seconds + 1.5) is False
    assert not any(harness.phase(i, 5, "STOPPED") for i in harness.ids)
    first_sample = min(row["stamp"] for rows in harness.rows.values() for row in rows)
    return dict(wall_s=monotonic() - harness.job_start, first_sample_stamp=first_sample)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, choices=[1, 3], default=3)
    parser.add_argument("--algorithm", choices=["nonlinear", "pid"], default="nonlinear")
    parser.add_argument("--implementation", choices=["cpp", "python"], default="cpp")
    parser.add_argument("--pressure-seconds", type=float, default=0.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    harness = Harness(args.robots, args.algorithm, args.implementation)
    report = {"implementation": args.implementation, "algorithm": args.algorithm,
              "robots": args.robots}
    try:
        if args.pressure_seconds:
            report.update(stress(harness, args.pressure_seconds))
        else:
            exercise(harness)
        report["passed"] = True
        report["summary"] = {}
        for i, rows in harness.rows.items():
            tracking = [row for row in rows if row.get("phase") == "TRACKING" and "compute_s" in row]
            samples = [row["compute_s"] * 1000. for row in tracking]
            intervals = [row["callback_interval_s"] for row in tracking
                         if "callback_interval_s" in row]
            assert samples
            summary = report["summary"][i] = dict(
                samples=len(samples), p50_ms=percentile(samples, .5),
                p95_ms=percentile(samples, .95), p99_ms=percentile(samples, .99),
                max_ms=max(samples))
            if intervals:
                summary.update(callback_interval_p99_ms=1000. * percentile(intervals, .99),
                               callback_interval_max_ms=1000. * max(intervals),
                               intervals_above_40ms=sum(v > .040 for v in intervals),
                               intervals_at_least_80ms=sum(v >= .080 for v in intervals))
        print(json.dumps(report, indent=2), flush=True)
    finally:
        try:
            args.output.write_text(json.dumps({**report, "status_rows": harness.rows},
                                              separators=(",", ":"), allow_nan=False) + "\n")
        finally:
            harness.teardown()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
