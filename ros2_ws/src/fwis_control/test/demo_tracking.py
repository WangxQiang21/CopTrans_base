#!/usr/bin/env python3
"""Run and visualize one feasible single-Ranger tracking trajectory."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from time import monotonic, sleep

import yaml

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from fwis_v3_msgs.msg import TimedTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from ros_fixtures import trajectory_message
from scenarios import A, O, S, Leg, sample_case


TOPIC = "/ranger_demo/timed_trajectory"
REFERENCE = "/ranger1/demo/reference_path"
ACTUAL = "/ranger1/demo/actual_path"
DISPLAY_PATH_HEIGHT = .42
MODES = {0: "Ackermann（直行和转弯）", 1: "Parallel（横移）", 2: "Spin（原地自旋）"}
FAILED_PHASES = {"STOPPED", "REJECTED", "STALE_TRAJECTORY"}


def demo_trajectory():
    return sample_case("demo_tracking", [
        Leg(A, .55, .17),
        Leg(A, .79, .17, parameter=1.),
        Leg(O, .56, .16, parameter=math.pi / 2),
        Leg(S, math.pi / 2, .43),
        Leg(A, .42, .15),
    ], plan_id=700).trajectory


def write_rviz_config(directory):
    description = Path(get_package_share_directory("ranger_mini_description"))
    config = yaml.safe_load((description / "rviz" / "ranger_tracking.rviz").read_text())
    manager = config["Visualization Manager"]
    manager["Views"]["Current"].update(
        {"Target Frame": "<Fixed Frame>", "Distance": 3.75, "Pitch": .75,
         "Focal Point": {"X": .68, "Y": .46, "Z": .27}})
    # Give the narrower actual trace a 3 mm rendering offset where the lines coincide.
    for name, topic, color, width, height in (
        ("Reference (blue)", REFERENCE, "64; 155; 255", .036, DISPLAY_PATH_HEIGHT),
        ("Actual (orange)", ACTUAL, "255; 170; 57", .014, DISPLAY_PATH_HEIGHT + .003),
    ):
        manager["Displays"].append({
            "Class": "rviz_default_plugins/Path", "Name": name, "Enabled": True,
            "Topic": {"Value": topic, "Reliability Policy": "Reliable",
                      "Durability Policy": "Transient Local", "Depth": 1},
            "Color": color, "Alpha": 1., "Line Style": "Billboards",
            "Line Width": width, "Pose Style": "None", "Buffer Length": 1,
            "Offset": {"X": 0., "Y": 0., "Z": height},
        })
    dest = directory / "tracking_demo.rviz"
    dest.write_text(yaml.safe_dump(config, sort_keys=False))
    return dest


class Demo(Node):
    def __init__(self, trajectory):
        super().__init__("ranger_tracking_demo",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.trajectory = trajectory
        self.latest_odom_ns = 0
        self.last_trace_ns = 0
        self.record = False
        self.done = None
        self.reported_mode = None
        self.actual_path = RosPath()
        self.actual_path.header.frame_id = trajectory.frame
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.plan = self.create_publisher(TimedTrajectory, TOPIC, durable)
        self.reference = self.create_publisher(RosPath, REFERENCE, durable)
        self.actual = self.create_publisher(RosPath, ACTUAL, durable)
        self.create_subscription(Odometry, "/ranger1/odom", self.on_odom, 5)
        self.create_subscription(String, "/ranger1/controller_status", self.on_status, 10)

    def on_odom(self, message):
        if message.header.frame_id != self.trajectory.frame:
            return
        stamp = message.header.stamp
        self.latest_odom_ns = stamp.sec * 10**9 + stamp.nanosec
        if not self.record or self.latest_odom_ns - self.last_trace_ns < 100_000_000:
            return
        self.last_trace_ns = self.latest_odom_ns
        pose = PoseStamped(header=message.header)
        pose.pose.position.x = message.pose.pose.position.x
        pose.pose.position.y = message.pose.pose.position.y
        pose.pose.orientation = message.pose.pose.orientation
        self.actual_path.header.stamp = stamp
        self.actual_path.poses.append(pose)
        self.actual.publish(self.actual_path)

    def on_status(self, message):
        row = json.loads(message.data)
        if not self.record or row.get("plan_id") != self.trajectory.plan_id:
            return
        if row["kind"] == "admission":
            print(f"轨迹预检查 {row['result']}")
        mode = row.get("feedback_mode")
        if mode in MODES and mode != self.reported_mode:
            self.reported_mode = mode
            print(f"底盘反馈模式 {MODES[mode]}")
        phase = row.get("phase")
        if phase in FAILED_PHASES or phase == "COMPLETE":
            self.done = row
            self.record = False

    def publish_reference(self):
        path = RosPath()
        path.header.frame_id = self.trajectory.frame
        path.header.stamp = self.get_clock().now().to_msg()
        for point in self.trajectory.points:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x = point.pose.x
            pose.pose.position.y = point.pose.y
            pose.pose.orientation.z = math.sin(point.pose.yaw / 2)
            pose.pose.orientation.w = math.cos(point.pose.yaw / 2)
            path.poses.append(pose)
        self.reference.publish(path)


def start_process(children, name, command, directory):
    with (directory / f"{name}.log").open("w") as log:
        child = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL,
                                 stdout=log, stderr=subprocess.STDOUT)
    children.append((name, child))


def check_processes(children):
    for name, child in children:
        if child.poll() is not None:
            raise RuntimeError(f"{name} 已退出（状态 {child.returncode}）")


def active_groups(children, timeout):
    deadline = monotonic() + timeout
    while True:
        active = []
        for child in children:
            child.poll()
            try:
                os.killpg(child.pid, 0)
                active.append(child)
            except ProcessLookupError:
                pass
        if not active or monotonic() >= deadline:
            return active
        sleep(.05)


def stop_processes(children):
    # Give ros2 launch one interrupt so it can stop its own nodes in order.
    # Group signals are only a fallback for processes still left behind.
    processes = [child for _, child in children]
    for child in processes:
        child.send_signal(signal.SIGINT)
    active = active_groups(processes, 5.)
    for signum, timeout in ((signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
        for child in active:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass
        active = active_groups(active, timeout)


def wait_until(node, children, condition, timeout, failure):
    deadline = monotonic() + timeout
    while not condition():
        check_processes(children)
        if monotonic() >= deadline:
            raise RuntimeError(failure)
        rclpy.spin_once(node, timeout_sec=.10)


def run(args):
    if not args.headless:
        try:
            get_package_share_directory("fwis_rviz_plugins")
        except PackageNotFoundError as exc:
            raise RuntimeError("RViz 面板还未构建，请先运行 colcon build --packages-select fwis_rviz_plugins") from exc
    directory = Path(tempfile.mkdtemp(prefix="ranger-tracking-demo-"))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1",
                      GZ_PARTITION=f"whut_ranger_demo_{os.getpid()}",
                      ROS_LOG_DIR=str(directory / "ros"))
    print(f"控制器 {args.algorithm}，本机 ROS 域 {args.domain_id}。日志在 {directory}")
    children = []
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Demo(demo_trajectory())
        start_process(children, "Gazebo", [
            "ros2", "launch", "--noninteractive", "fwis_control", "tracking.launch.py",
            f"algorithm:={args.algorithm}", f"trajectory_topic:={TOPIC}",
            f"gui:={'false' if args.headless else 'true'}", "rviz:=false",
        ], directory)
        if not args.headless:
            start_process(children, "RViz", [
                "rviz2", "-d", str(write_rviz_config(directory)),
                "--ros-args", "-p", "use_sim_time:=true",
            ], directory)
        wait_until(node, children, lambda: (
            node.latest_odom_ns > 0 and node.get_clock().now().nanoseconds > 0
            and node.plan.get_subscription_count() > 0
            and (args.headless or node.reference.get_subscription_count() > 0)
        ), 45., "等待 Gazebo 里程计、控制器或 RViz 订阅超时")
        node.publish_reference()
        print("演示顺序为直行 → 左转 → 左横移 → 原地自旋 90° → 直行停车。")
        auto = args.auto_start or args.headless or not sys.stdin.isatty()
        if not auto:
            input("安排好两个窗口后，回到终端按回车开始，Ctrl+C 退出。")
        # Refresh the simulation clock after the user arranges the windows.
        ready_ns = node.get_clock().now().nanoseconds
        wait_until(node, children, lambda: (
            node.get_clock().now().nanoseconds > ready_ns + 150_000_000
            and abs(node.get_clock().now().nanoseconds - node.latest_odom_ns) < 150_000_000
        ), 10., "仿真时钟未推进。请检查 Gazebo 是否暂停")
        node.record = True
        node.plan.publish(trajectory_message(node.trajectory, node.get_clock().now().to_msg()))
        print(f"开始跟踪，参考时长 {node.trajectory.duration:.1f} 仿真秒。")
        wait_until(node, children, lambda: node.done is not None, 150., "等待轨迹完成超时")
        if node.done["phase"] != "COMPLETE":
            reasons = "、".join(node.done.get("reasons", [])) or node.done["phase"]
            raise RuntimeError(f"跟踪停止（{reasons}）")
        ex, ey, yaw = node.done["body_error"]
        print(f"COMPLETE。终点位置误差 {100 * math.hypot(ex, ey):.2f} cm，"
              f"航向误差 {abs(math.degrees(yaw)):.2f}°。实际轨迹 {len(node.actual_path.poses)} 个采样点。")
        if not args.headless:
            print("车已停稳，可以继续查看两个窗口。Ctrl+C 关闭。")
            while all(child.poll() is None for _, child in children):
                rclpy.spin_once(node, timeout_sec=.2)
    finally:
        stop_processes(children)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def interrupt_demo(_signum, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--algorithm", choices=("nonlinear", "pid"), default="nonlinear")
    parser.add_argument("--auto-start", action="store_true", help="Skip the Enter prompt")
    parser.add_argument("--headless", action="store_true", help="Run without windows and exit on completion")
    parser.add_argument("--domain-id", type=int, default=177, help="Isolated local ROS domain")
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be between 0 and 232")
    signal.signal(signal.SIGTERM, interrupt_demo)
    signal.signal(signal.SIGHUP, interrupt_demo)
    lock = Path(tempfile.gettempdir()) / f"ranger-tracking-demo-{os.getuid()}-{args.domain_id}.lock"
    try:
        with lock.open("a") as handle:
            try:
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("这个 ROS 域已有演示在运行，请关闭它或指定 --domain-id") from exc
            run(args)
    except (KeyboardInterrupt, EOFError):
        return 0
    except (OSError, RuntimeError) as exc:
        print(f"演示结束，{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
