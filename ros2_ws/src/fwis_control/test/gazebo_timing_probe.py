#!/usr/bin/env python3
"""Measure actual Gazebo transport on unchanged Twist and odometry topics."""

import argparse
from contextlib import ExitStack
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from time import monotonic

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from fwis_v3_msgs.msg import FormationExecutionPermit, TimedTrajectory
from geometry_msgs.msg import Twist as RosTwist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Int32, String

from fwis_control.model import Pose
from ranger_mini_gz.scene_generator import clearance_summary, load_scene
from ros_fixtures import trajectory_message
from scenarios import A, Leg, sample_case


SCENE = "factory_indoor_01"
RADIUS_M = 1.12
SPEED_MPS = .14
PLAN_ID = 10901


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8.)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def route_table(scene, task, minimum_wall_s):
    spawns = scene.spawn_poses(task)
    fixture = sample_case("control_timing_circle", [Leg(
        A, SPEED_MPS * (minimum_wall_s + 40.), SPEED_MPS, parameter=1. / RADIUS_M)],
        plan_id=PLAN_ID)
    local = fixture.trajectory
    routes = {f"robot{i}": replace(local, points=tuple(
        replace(p, pose=Pose(p.pose.x+x, p.pose.y+y, p.pose.yaw))
        for p in local.points)) for i, (x, y, _) in spawns.items()}
    fx, fy, base_yaw = scene.experiments[task]["start"]
    assert base_yaw == 0. and all(s[2] == 0. for s in spawns.values())
    audit_rows = []
    for i in range(181):
        theta = math.tau * i / 180.
        x, y = RADIUS_M * math.sin(theta), RADIUS_M * (1. - math.cos(theta))
        bodies = [(px+x, py+y, theta) for px,py,_ in spawns.values()]
        formation = (fx+x,fy+y,0.)
        audit_rows.append((formation,bodies) if task == "formation" else (bodies[0], None))
    clearance = {key: row["distance_m"] for key, row in
                 clearance_summary(scene, task, audit_rows).items()}
    if min(clearance.values()) < .15:
        raise ValueError(f"the selected pressure fixture lacks static clearance: {clearance}")
    return routes, clearance


class Probe(Node):
    def __init__(self, names, output, coordinated, stack):
        super().__init__("native_gazebo_control_timing",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.names = names
        self.routes = {}
        self.ready_topics = set()
        self.rows = {}
        self.faults = {}
        self.recording = False
        self.admitted = set()
        self.stamp = None
        self.enabled = True
        self.status_files = {}
        for name in names:
            self.rows[name] = 0
            self.status_files[name] = stack.enter_context((output / f"{name}_controller.jsonl").open("w"))
            topics = ((Odometry, "odom", 5), (Int32, "motion_state", 5), (RosTwist, "cmd_vel", 5))
            for kind, topic, depth in topics:
                endpoint = f"/{name}/{topic}"
                self.create_subscription(kind, endpoint, lambda _, j=endpoint: self.ready_topics.add(j), depth)
            self.create_subscription(String, f"/{name}/controller_status",
                                     lambda msg, robot=name: self.on_status(robot,msg), 20)
            self.routes[name] = self.create_publisher(
                TimedTrajectory, f"/{name}/trajectory",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.permit = None
        if coordinated:
            self.permit = self.create_publisher(FormationExecutionPermit, "/formation/execution_permit", 1)
            self.create_timer(.03, self.send_permit, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_status(self, robot, message):
        row = json.loads(message.data)
        if row.get("plan_id") != PLAN_ID:
            return
        if row.get("kind") == "admission":
            if row["result"] in ("PASS", "WARN"):
                self.admitted.add(robot)
            elif self.recording:
                self.faults[robot] = row
        if self.recording and row.get("kind") == "execution" and row.get("phase") == "TRACKING":
            self.rows[robot] += 1
        if self.recording and row.get("phase") in ("STOPPED", "REJECTED", "STALE_TRAJECTORY"):
            self.faults[robot] = row
        self.status_files[robot].write(message.data + "\n")

    def send_permit(self):
        if (self.permit is not None and self.stamp is not None and
                set(self.names) == self.admitted):
            self.permit.publish(FormationExecutionPermit(
                stamp=self.get_clock().now().to_msg(), plan_id=PLAN_ID,
                start=self.stamp, enabled=self.enabled))

    def publish_routes(self, routes):
        # The standalone single-car controller only admits within clock tolerance.
        lead_ns = 1_350_000_000 if self.permit is not None else 0
        start = self.get_clock().now().nanoseconds + lead_ns
        self.stamp = Time(nanoseconds=start).to_msg()
        for robot, route in routes.items():
            self.routes[robot].publish(trajectory_message(route, self.stamp))

    def ready(self):
        return (self.get_clock().now().nanoseconds > 0 and
                len(self.ready_topics) == len(self.names)*3 and
                all(p.get_subscription_count() for p in self.routes.values()) and
                all(self.count_publishers(f"/{name}/controller_status") for name in self.names) and
                (self.permit is None or self.permit.get_subscription_count() == len(self.names)))

    def spin_until(self, predicate, processes, timeout_s):
        deadline = monotonic() + timeout_s
        while monotonic() < deadline and not predicate():
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("a Gazebo or tracker process exited before measurement ended")
            if self.faults:
                raise RuntimeError(f"control aborted: {self.faults}")
            rclpy.spin_once(self, timeout_sec=.025)
        return predicate()


def launch(command, env, output, tag, stack):
    log = stack.enter_context((output / f"{tag}.log").open("w"))
    process = subprocess.Popen(command, env=env, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    stack.callback(stop, process)
    return process


def scheduler_visibility():
    values = {}
    for name in ("perf_event_paranoid", "sched_schedstats"):
        path = Path("/proc/sys/kernel") / name
        try:
            values[name] = int(path.read_text())
        except (OSError, ValueError):
            values[name] = None
    return {**values, "thread_observations": "cpu_time_and_context_switch_counts",
            "kernel_runnable_intervals_recorded": False}


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--robots", type=int, choices=(1, 3), required=True)
    cli.add_argument("--dds-domain", type=int, required=True)
    cli.add_argument("--seconds", type=float, default=605.)
    cli.add_argument("--min-samples", type=int, default=15_000)
    cli.add_argument("--output", type=Path, required=True)
    cli.add_argument("--algorithm", choices=("nonlinear", "pid"), default="nonlinear")
    cli.add_argument("--trace-ros-bridge", action="store_true",
                     help="Capture command takes inside the existing ROS/Gazebo bridge")
    args = cli.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(args.dds_domain),
                      GZ_PARTITION=f"coptrans_timing_{args.dds_domain}",
                      ROS_LOG_DIR=str(args.output / "ros"), COPTRANS_TIMING_DIR=str(args.output))
    coordinated = args.robots == 3
    task = "formation" if coordinated else "single"
    routes, clearance = route_table(load_scene(SCENE), task, args.seconds)
    library_dir = Path(get_package_prefix("fwis_control")) / "lib/fwis_control"
    executable = str(library_dir / "ranger_tracking_controller_cpp")
    controller_path = args.output / "controller_parameters.yaml"
    shutil.copyfile(Path(get_package_share_directory("fwis_control")) / "config/ranger_tracking.yaml",
                    controller_path)
    environment = dict(os.environ)
    if args.trace_ros_bridge:
        trace = library_dir / "libcoptrans_bridge_take_trace.so"
        if not trace.is_file():
            raise FileNotFoundError(f"bridge trace library missing: {trace}")
        environment.update(COPTRANS_TRACE_RCL_TAKE_SO=str(trace))
    rclpy.init(args=[])
    report = dict(robots=args.robots, requested_wall_s=args.seconds, minimum_samples=args.min_samples,
                  scenario=SCENE, planned_sampled_clearance_m=clearance,
                  algorithm=args.algorithm, configuration="baseline, uninterrupted A circle",
                  tracker_executable=executable, bridge_take_trace=args.trace_ros_bridge,
                  invocation=sys.argv, scheduler_visibility=scheduler_visibility(), status="failed")
    try:
        with ExitStack() as stack:
            probe = Probe(tuple(routes), args.output, coordinated, stack)
            stack.callback(probe.destroy_node)
            process = launch([
                "ros2", "launch", "fwis_bringup", "simulation.launch.py", f"scenario:={SCENE}",
                f"task:={task}", "gui:=false", "rviz:=false", "body_twist:=true",
                f"scene_directory:={args.output / 'scene'}"],
                environment, args.output, "gazebo", stack)
            processes = [process]
            for robot in routes:
                parameters = ["-p", "use_sim_time:=true", "-p", f"algorithm:={args.algorithm}",
                              "-p", f"config_file:={controller_path}",
                              "-p", f"trajectory_topic:=/{robot}/trajectory"]
                if coordinated:
                    parameters += ["-p", "permit_topic:=/formation/execution_permit",
                                   "-p", "future_start_window_s:=2.0"]
                processes.append(launch([executable, "--ros-args", "-r", f"__ns:=/{robot}",
                                        *parameters], environment, args.output, robot, stack))

            if not probe.spin_until(probe.ready, processes, 35.):
                raise RuntimeError(f"Gazebo and native tracker topics were not ready: {probe.ready_topics}")
            probe.spin_until(lambda: False, processes, .30)
            probe.recording = True
            probe.publish_routes(routes)
            if not probe.spin_until(lambda: set(routes) == probe.admitted, processes, 3.):
                raise RuntimeError(f"admission not received: {probe.admitted}")
            if not probe.spin_until(lambda: all(count > 0 for count in probe.rows.values()), processes, 5.):
                raise RuntimeError(f"trackers did not engage: {probe.rows}")
            first_wall = monotonic()
            first_sim = probe.get_clock().now().nanoseconds
            print(f"TRACKING {args.robots} cars, pressure clock started", flush=True)
            passed = probe.spin_until(
                lambda: (monotonic()-first_wall >= args.seconds and
                         min(probe.rows.values()) >= args.min_samples),
                processes, max(20.,args.seconds*1.8))
            no_motion_faults = not probe.faults
            report.update(
                tracking_wall_s=monotonic()-first_wall,
                tracking_sim_s=(probe.get_clock().now().nanoseconds-first_sim)*1e-9,
                samples=probe.rows, faults=probe.faults,
                status="passed" if passed and no_motion_faults else "failed")
            probe.recording = False
            if coordinated:
                probe.enabled = False
            else:
                message = TimedTrajectory(plan_id=PLAN_ID)
                message.header.frame_id = "map"
                message.header.stamp = probe.get_clock().now().to_msg()
                probe.routes["robot1"].publish(message)
            probe.spin_until(lambda: False, processes, .44)
            print(json.dumps(report, indent=2), flush=True)
    finally:
        (args.output / "probe_summary.json").write_text(json.dumps(report, indent=2) + "\n")
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
