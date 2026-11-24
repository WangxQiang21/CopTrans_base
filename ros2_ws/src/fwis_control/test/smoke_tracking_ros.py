#!/usr/bin/env python3
"""Exercise the installed tracking launch through ROS 2 and Harmonic."""
import argparse
import json
import math
from pathlib import Path
import sys
from time import monotonic

from fwis_v3_msgs.msg import TimedTrajectory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from ros_fixtures import trajectory_message
from scenarios import A, O, S, Leg, sample_case, suite

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fwis_bringup/test"))
from smoke_plan import DURABLE, fixture_map


def pose_message(node, x, y, yaw):
    message = PoseStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "map"
    message.pose.position.x, message.pose.position.y = x, y
    message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-planner", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node("tracking_ros_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
    odometry, commands, statuses, receipts = [], [], [], []
    node.create_subscription(Odometry, "/ranger1/odom", odometry.append, 5)
    node.create_subscription(Twist, "/ranger1/cmd_vel", commands.append, 5)
    node.create_subscription(String, "/ranger1/controller_status",
                             lambda msg: statuses.append(json.loads(msg.data)), 10)
    node.create_subscription(TimedTrajectory, "/plan/timed_trajectory",
                             lambda msg: receipts.append(msg) if msg.points else None, DURABLE)
    pub = node.create_publisher(TimedTrajectory, "/plan/timed_trajectory", DURABLE)

    def wait_until(predicate, duration):
        deadline = monotonic()+duration
        while not predicate() and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.08)
        return predicate()

    try:
        assert wait_until(lambda: odometry and commands and node.get_clock().now().nanoseconds > 0
                          and pub.get_subscription_count() >= 2, 20.)
        if args.native_planner:
            grid = node.create_publisher(OccupancyGrid, "/map", DURABLE)
            start_pub = node.create_publisher(PoseStamped, "/start_pose", 10)
            goal_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
            assert wait_until(lambda: grid.get_subscription_count() >= 2
                              and goal_pub.get_subscription_count() > 0, 20.)
            grid.publish(fixture_map(node, False))
            wait_until(lambda: False, .8)
            start_pub.publish(pose_message(node, 3., 4., 0.))
            wait_until(lambda: False, .6)
            goal_pub.publish(pose_message(node, 4., 4.75, -1.1))
        else:
            invalid = next(case for case in suite() if case.name == "reject_tight_radius")
            pub.publish(trajectory_message(invalid.trajectory, node.get_clock().now().to_msg()))
            assert wait_until(lambda: any(row.get("result") == "FAIL" and "ackermann_radius" in
                              {i["code"] for i in row.get("issues", [])} for row in statuses), 5.)
            assert all(max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z)) <= 1e-9
                       for msg in commands)
            valid = sample_case("ros_mixed", [Leg(A, .30, .13), Leg(O, .18, .12, parameter=.7),
                                             Leg(S, .46, .29), Leg(A, .22, .12)], plan_id=500)
            pub.publish(trajectory_message(valid.trajectory, node.get_clock().now().to_msg()))

        assert wait_until(lambda: bool(receipts) and
                          (args.native_planner or receipts[-1].plan_id == 500), 30.), statuses[-4:]
        message = receipts[-1]
        expected_modes = {int(p.mode) for p in message.points}
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

        def matching_rows():
            same_key = [row for row in statuses
                        if (row.get("plan_id"), row.get("trajectory_stamp_ns")) == (message.plan_id, stamp)]
            # CopTrans can clear and publish under the same simulated time
            # stamp while a planning callback owns the executor.
            for index, row in enumerate(same_key):
                if row["kind"] == "admission" and row.get("result") != "CLEARED":
                    return same_key[index:]
            return []

        def terminal():
            return [row for row in matching_rows() if row.get("phase")
                    in ("COMPLETE", "STOPPED", "REJECTED", "STALE_TRAJECTORY")]

        assert wait_until(terminal, 60.), statuses[-5:]
        finished = terminal()[0]
        assert finished["phase"] == "COMPLETE", finished
        observed = {i.get("feedback_mode") for i in matching_rows()}
        assert expected_modes.issubset(observed), (expected_modes, observed)
        events = [i["requested_mode"] for i in matching_rows() if i.get("phase") == "REQUEST_MODE_SWITCH"]
        assert 1 in events and 2 in events, events
        final = odometry[-1]
        assert abs(final.pose.pose.position.z-.322)<1e-7
        goal = message.points[-1].center_pose
        distance = math.hypot(final.pose.pose.position.x-goal.x, final.pose.pose.position.y-goal.y)
        print(f"plan {message.plan_id}: COMPLETE, modes {events}, final position error {distance:.5f} m",
              flush=True)
    finally:
        with args.output.open("w") as file:
            for row in statuses:
                file.write(json.dumps(row, allow_nan=False) + "\n")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
