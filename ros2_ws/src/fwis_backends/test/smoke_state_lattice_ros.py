#!/usr/bin/env python3
"""Check first-plan publication, tracker preflight, and invalidation on map changes."""

import importlib.util
import math
from pathlib import Path as FilePath
from time import monotonic

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy import init, try_shutdown
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fwis_control.config import load
from fwis_control.ros_state import read_trajectory
from fwis_control.trajectory import preflight
from fwis_v3_msgs.msg import TimedTrajectory


WORKSPACE_SRC = FilePath(__file__).resolve().parents[2]
path = WORKSPACE_SRC / "fwis_backends/single_continuous_mode_path/scripts/single_state_lattice_planner.py"
spec = importlib.util.spec_from_file_location("lattice_ros_test", path)
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def spin_until(executor, predicate, limit=5.0):
    started = monotonic()
    while not predicate():
        if monotonic() - started >= limit:
            raise AssertionError("smoke test wait exceeded deadline")
        executor.spin_once(timeout_sec=.02)


def main():
    init()
    planner = adapter.LatticePlanner()
    probe = Node("state_lattice_smoke_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(planner)
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    plans, statuses = [], []
    probe.create_subscription(TimedTrajectory, "/plan/timed_trajectory", plans.append, qos)
    probe.create_subscription(String, "/plan/status", lambda msg: statuses.append(msg.data), qos)
    map_pub = probe.create_publisher(OccupancyGrid, "/map", qos)
    start_pub = probe.create_publisher(PoseStamped, "/start_pose", 10)
    goal_pub = probe.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
    try:
        spin_until(executor, lambda: all(pub.get_subscription_count()
                                         for pub in (map_pub, start_pub, goal_pub)))
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.width = grid.info.height = 100
        grid.info.resolution = .05
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * (grid.info.width * grid.info.height)
        map_pub.publish(grid)
        spin_until(executor, lambda: any(s == "map_ready 100x100" for s in statuses))
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.orientation.w = 1.
        start.pose.position.x = start.pose.position.y = 2.
        start_pub.publish(start)
        spin_until(executor, lambda: planner.start is not None)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.orientation.w = 1.
        goal.pose.position.x = 2.
        goal.pose.position.y = 3.
        goal_pub.publish(goal)
        spin_until(executor, lambda: any(message.points for message in plans))
        plan = next(message for message in plans if message.points)
        endpoint = plan.points[-1].center_pose
        assert abs(endpoint.x - 2.) < 1e-8 and abs(endpoint.y - 3.) < 1e-8
        assert plan.points[-1].mode == 1
        cfg = load(WORKSPACE_SRC / "fwis_control/config/ranger_tracking.yaml")
        checked = preflight(read_trajectory(plan), cfg.limits, cfg.preflight, cfg.timing.minimum_hold)
        assert checked.result in ("PASS", "WARN"), checked.issues
        assert sum(bool(message.points) for message in plans) == 1

        # Distinct map replaces the request's map snapshot, even if the
        # changed cell lies far from the incumbent footprint.
        grid.data[0] = 100
        previous_count = len(plans)
        map_pub.publish(grid)
        spin_until(executor, lambda: len(plans) > previous_count)
        assert plans[-1].plan_id == plan.plan_id and not plans[-1].points

        angle, radius = .6, 2.
        goal.pose.position.x = 2. + radius * math.sin(angle)
        goal.pose.position.y = 2. + radius * (1. - math.cos(angle))
        goal.pose.orientation.z = math.sin(angle / 2.)
        goal.pose.orientation.w = math.cos(angle / 2.)
        goal_pub.publish(goal)
        spin_until(executor, lambda: sum(bool(message.points) for message in plans) == 2)
        second = next(message for message in reversed(plans) if message.points)
        assert second.plan_id != plan.plan_id
        assert all(point.mode == 0 for point in second.points)
        checked = preflight(read_trajectory(second), cfg.limits, cfg.preflight, cfg.timing.minimum_hold)
        assert checked.result in ("PASS", "WARN"), checked.issues

        goal.header.frame_id = "odom"
        goal_pub.publish(goal)
        spin_until(executor, lambda: any(s.startswith("invalid_goal_frame ") for s in statuses))
        assert sum(bool(message.points) for message in plans) == 2
        print("PASS state lattice A/O ROS preflight, first-cert publication, map invalidation and invalid frame")
    finally:
        planner.supersede()
        planner.pool.shutdown(wait=True, cancel_futures=True)
        executor.shutdown()
        probe.destroy_node()
        planner.destroy_node()
        try_shutdown()


if __name__ == "__main__":
    main()
