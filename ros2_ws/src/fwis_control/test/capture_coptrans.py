#!/usr/bin/env python3
"""Capture unchanged CopTrans TimedTrajectory messages from an isolated planner."""
import argparse
import json
import math
from pathlib import Path
import sys

from fwis_v3_msgs.msg import TimedTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.serialization import serialize_message
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "fwis_bringup/test"))
from smoke_plan import DURABLE, fixture_map, spin_for


def nanoseconds(message):
    return message.sec * 10**9 + message.nanosec


def capture(args):
    rclpy.init()
    node = rclpy.create_node("capture_coptrans_ranger")
    data = {}

    def receive(message):
        if message.points:
            goal = message.points[-1].center_pose
            distance = math.hypot(goal.x - args.goal[0], goal.y - args.goal[1])
            delta = goal.theta - args.goal[2]
            if distance < .03 and abs(math.atan2(math.sin(delta), math.cos(delta))) < .02:
                data["trajectory"] = message

    node.create_subscription(TimedTrajectory, "/plan/timed_trajectory", receive, DURABLE)
    node.create_subscription(String, "/plan/status", lambda msg: data.update(status=msg.data), DURABLE)
    map_pub = node.create_publisher(OccupancyGrid, "/map", DURABLE)
    start_pub = node.create_publisher(PoseStamped, "/start_pose", 10)
    goal_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

    def pose(values):
        x, y, yaw = values
        message = PoseStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return message

    try:
        if not spin_for(node, lambda: (goal_pub.get_subscription_count() > 0
                                       and map_pub.get_subscription_count() >= 2), 20.):
            raise RuntimeError("isolated CopTrans planner unavailable")
        args.output.mkdir(parents=True, exist_ok=True)
        map_pub.publish(fixture_map(node, False))
        spin_for(node, lambda: False, 1.)
        start_pub.publish(pose(args.start))
        spin_for(node, lambda: False, .6)
        sent = node.get_clock().now().nanoseconds
        goal_pub.publish(pose(args.goal))
        ok = spin_for(node, lambda: ("trajectory" in data and
                                     nanoseconds(data["trajectory"].header.stamp) >= sent), args.timeout)
        if not ok:
            failure = json.dumps({"name": args.name, "status": data.get("status")})
            (args.output / f"{args.name}_capture.json").write_text(failure + "\n")
            raise RuntimeError(f"no fresh trajectory reached requested goal: {failure}")
        trajectory = data["trajectory"]
        points = [dict(t_ns=nanoseconds(p.time_from_start), x=p.center_pose.x,
                       y=p.center_pose.y, yaw=p.center_pose.theta, v=p.v, mode=p.mode)
                  for p in trajectory.points]
        report = dict(name=args.name, source="coptrans", frame_id=trajectory.header.frame_id,
                      stamp_ns=nanoseconds(trajectory.header.stamp), plan_id=trajectory.plan_id,
                      points=points, provenance={"backend": "single_continuous_mode_path",
                                                 "profile": args.profile, "start": args.start, "goal": args.goal})
        (args.output / f"{args.name}.cdr").write_bytes(serialize_message(trajectory))
        (args.output / f"{args.name}.json").write_text(json.dumps(report, allow_nan=False) + "\n")
        print(f"{args.name}: {len(points)} points, plan_id={trajectory.plan_id}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile", default="default")
    parser.add_argument("--start", type=float, nargs=3, default=(3., 4., 0.))
    parser.add_argument("--goal", type=float, nargs=3, required=True)
    parser.add_argument("--timeout", type=float, default=40.)
    args = parser.parse_args()
    if not args.name or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789_-" for c in args.name):
        parser.error("--name must use lowercase letters, digits, underscores or hyphens")
    capture(args)


if __name__ == "__main__":
    main()
