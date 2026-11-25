#!/usr/bin/env python3
"""Exercise production launch and ROS planning topics on two matched backends."""

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
from tempfile import TemporaryDirectory
from time import monotonic

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import yaml

from fwis_backends.single_continuous_mode_path.core import Pose
from fwis_v3_msgs.msg import TimedTrajectory
from ranger_mini_gz.scene_generator import read_map


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "ros2_ws/src"
ARCHIVE = ROOT / "reports/runs/20261124_213326_single_factory_indoor_01_continuous_mode_path_dqKCHS"


def cases():
    module_path = SRC / "fwis_backends/test/test_state_lattice.py"
    spec = importlib.util.spec_from_file_location("lattice_fixture", module_path)
    fixture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture)
    for name, start, goal, obstacles, reverse, _success, _modes in fixture.LATTICE_CASES:
        if reverse:
            grid = fixture.make_grid(obstacles)
            yield name, start, goal, grid.occ, grid.res, grid.ox, grid.oy
    config = yaml.safe_load((ARCHIVE / "resolved_config.yaml").read_text())
    meta, cells = read_map(ARCHIVE / "scene/map.yaml")
    yield ("indoor_full_route", Pose(*config["start"]), Pose(*config["goal"]),
           cells != 0, meta["resolution"], *meta["origin"][:2])


class Probe(Node):
    def __init__(self):
        super().__init__("phase5_portfolio_probe")
        qos = QoSProfile(depth=32, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos)
        self.start_pub = self.create_publisher(PoseStamped, "/start_pose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.ready = {"search": 0, "optimizer": 0}
        self.plans, self.status, self.trace = {}, {}, {}
        self.create_subscription(TimedTrajectory, "/plan/timed_trajectory", self.on_plan, qos)
        for topic, slot in (("/plan/search_status", "search"), ("/plan/status", "optimizer")):
            self.create_subscription(String, topic, lambda msg, s=slot: self.on_status(msg, s), qos)
        self.create_subscription(String, "/plan/lattice_trace", self.on_trace, qos)

    def on_plan(self, message):
        if message.points:
            self.plans.setdefault(message.plan_id, []).append((monotonic(), message))

    def on_status(self, message, slot):
        text = message.data
        if text.startswith("map_ready "):
            self.ready[slot] += 1
        for word in text.split():
            if word.startswith("id=") and word[3:].isdigit():
                self.status.setdefault(int(word[3:]), []).append(text)
                return

    def on_trace(self, message):
        data = json.loads(message.data)
        self.trace.setdefault(data["plan_id"], []).append(data)

    def wait(self, condition, timeout):
        end = monotonic() + timeout
        while monotonic() < end:
            if condition():
                return True
            rclpy.spin_once(self, timeout_sec=.03)
        return False

    def pose(self, value):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(value.x), float(value.y)
        msg.pose.orientation.z = math.sin(value.th / 2.)
        msg.pose.orientation.w = math.cos(value.th / 2.)
        return msg

    def evaluate(self, index, case):
        name, start, target, obstacles, resolution, origin_x, origin_y = case
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.height, grid.info.width = obstacles.shape
        grid.info.resolution = resolution
        grid.info.origin.position.x, grid.info.origin.position.y = origin_x, origin_y
        grid.info.origin.orientation.w = 1.
        grid.data = (obstacles.flatten().astype(np.int8) * 100).tolist()
        old_ready = dict(self.ready)
        self.map_pub.publish(grid)
        assert self.wait(lambda: all(self.ready[key] > count for key, count in old_ready.items()), 20.), name
        self.start_pub.publish(self.pose(start))
        self.wait(lambda: False, .35)
        begin = monotonic()
        self.goal_pub.publish(self.pose(target))

        def completed():
            return self.plans.get(index) or any(status.startswith(
                ("search_no_path", "action_policy_rejected", "search_budget_exhausted",
                 "rejected", "geometry_only", "frontend_rejected"))
                for status in self.status.get(index, ()))

        deadline_hit = not self.wait(completed, 28.)
        self.wait(lambda: False, .25)
        outputs = self.plans.get(index, ())
        assert len(outputs) <= 1, (name, index, len(outputs))
        result = {"case": name, "id": index, "trajectory_count": len(outputs),
                  "timeout": deadline_hit, "status": self.status.get(index, []),
                  "trace": self.trace.get(index, [])}
        if outputs:
            received, trajectory = outputs[0]
            point = trajectory.points[-1]
            modes = []
            for p in trajectory.points:
                if modes[-1:] != [p.mode]:
                    modes.append(p.mode)
            result.update(request_to_wire_s=received - begin,
                          reference_duration_s=point.time_from_start.sec + point.time_from_start.nanosec * 1e-9,
                          terminal_position_error_m=math.hypot(point.center_pose.x - target.x,
                                                               point.center_pose.y - target.y),
                          terminal_yaw_error_rad=abs(math.remainder(point.center_pose.theta - target.th, 2 * math.pi)),
                          mode_word=modes)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--backend", choices=("single_state_lattice", "single_continuous_mode_path"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--warmups", type=int, default=0)
    parser.add_argument("--repeat-case", help="apply warmups and repetitions only to this named case")
    args = parser.parse_args()
    if args.repeat < 1 or args.warmups < 0:
        parser.error("expected a positive repetition count and nonnegative warmups")
    inputs = [case for case in cases() if not args.case or case[0] in args.case]
    if not inputs:
        raise ValueError("no matching scenario")
    baseline = yaml.safe_load((SRC / "fwis_bringup/config/test_defaults.yaml").read_text())["profiles"]["single"]
    output = {"backend": args.backend, "baseline_overrides": baseline, "scenarios": [],
              "repeat": args.repeat, "warmups": args.warmups, "repeat_case": args.repeat_case}
    with TemporaryDirectory(prefix="phase5-ros-") as tmp:
        overrides = Path(tmp) / "planner.yaml"
        overrides.write_text(yaml.safe_dump({**baseline, "quality": {
            **baseline["quality"], "report_dir": str(Path(tmp) / "optimizer")}}))
        with args.output.with_suffix(".launch.log").open("w") as log:
            command = ["ros2", "launch", "fwis_bringup", "plan.launch.py", "rviz:=false",
                       f"backend:={args.backend}", f"overrides_file:={overrides}"]
            launcher = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            probe = Probe()
            try:
                assert probe.wait(lambda: all(pub.get_subscription_count() for pub in
                                              (probe.map_pub, probe.start_pub, probe.goal_pub)), 30.)
                index = 0
                for case in inputs:
                    repeat, warmups = ((args.repeat, args.warmups)
                                       if not args.repeat_case or case[0] == args.repeat_case else (1, 0))
                    for trial in range(repeat + warmups):
                        index += 1
                        item = probe.evaluate(index, case)
                        item["warmup"] = trial < warmups
                        output["scenarios"].append(item)
                        args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
                        print(args.backend, item["case"], trial + 1, item.get("reference_duration_s"),
                              flush=True)
                # A completed selection must not produce a second trajectory
                # when a slower contender finishes later.
                probe.wait(lambda: False, .5)
                assert max(map(len, probe.plans.values()), default=0) <= 1
            finally:
                probe.destroy_node()
                rclpy.try_shutdown()
                if launcher.poll() is None:
                    os.killpg(launcher.pid, signal.SIGINT)
                try:
                    launcher.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(launcher.pid, signal.SIGTERM)
                    launcher.wait(timeout=5)


if __name__ == "__main__":
    main()
