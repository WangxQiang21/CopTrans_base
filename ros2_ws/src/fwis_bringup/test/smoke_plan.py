#!/usr/bin/env python3
"""Exercise a running CopTrans backend with a fixed open map and straight request."""

import argparse
import json
import math
from pathlib import Path as FilePath
import time

from fwis_v3_msgs.msg import FormationPlanRequest, FormationPlaybackPlan, FormationVehicleSpec, TimedTrajectory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

START = (3.0, 4.0)
GOAL = (5.8, 4.0)
OFFSETS = ((1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
DURABLE = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1_000_000_000.0


def fixture_map(node, blocked):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.header.stamp = node.get_clock().now().to_msg()
    grid.info.origin.orientation.w = 1.0
    grid.info.width = 200
    grid.info.height = 160
    grid.info.resolution = .1
    data = [0] * (grid.info.width * grid.info.height)
    if blocked:
        for row in range(24, 58):
            begin = row * grid.info.width + 53
            data[begin:begin + 25] = [100] * 25
    grid.data = data
    return grid


def pose(node, position):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x, message.pose.position.y = position
    message.pose.orientation.w = 1.0
    return message


def formation_request(node):
    message = FormationPlanRequest()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.plan_id = 600001
    message.formation_id = "cargo_wedge"
    message.formation_start.x, message.formation_start.y = START
    message.formation_goal.x, message.formation_goal.y = GOAL
    message.allow_reverse = True
    for index, (dx, dy) in enumerate(OFFSETS, start=1):
        vehicle = FormationVehicleSpec()
        vehicle.vehicle_id = index
        vehicle.offset_in_formation.x = dx
        vehicle.offset_in_formation.y = dy
        vehicle.start_center.x, vehicle.start_center.y = START[0] + dx, START[1] + dy
        vehicle.goal_center.x, vehicle.goal_center.y = GOAL[0] + dx, GOAL[1] + dy
        vehicle.goal_heading_free = True
        vehicle.body_length_m = .738
        vehicle.body_width_m = .500
        message.vehicles.append(vehicle)
    return message


def spin_for(node, condition, timeout):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    return condition()


def check_track(points, start, goal):
    assert len(points) > 2, "expected at least three trajectory samples"
    positions = [(point.center_pose.x, point.center_pose.y) for point in points]
    assert math.dist(positions[0], start) < 1e-3, positions[0]
    assert math.dist(positions[-1], goal) < .03, positions[-1]
    assert all(abs(point[1] - start[1]) < .08 for point in positions), "straight-case lateral deviation"
    # The box diagonal is smaller than 1 m. A 0.5 m guard keeps its complete
    # footprint inside the occupied-grid bounds along this open-map fixture.
    assert all(.5 < x < 19.5 and .5 < y < 15.5 for x, y in positions)
    assert all(abs(point.center_pose.theta) < .2 for point in points), "straight-case body heading"
    times = [stamp_seconds(point.time_from_start) for point in points]
    assert all(a < b for a, b in zip(times, times[1:])), "trajectory time must increase"
    assert all(point.mode == 0 for point in points), "expected Ackermann on straight case"
    return len(points), times[-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--multi", action="store_true")
    parser.add_argument("--timeout", type=float, default=95.0)
    parser.add_argument("--json-out", type=FilePath)
    parser.add_argument("--blocked", action="store_true", help="Put the goal footprints in occupied cells")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("coptrans_smoke")
    received = {}

    def subscribe(name, msg_type, topic, qos=DURABLE):
        node.create_subscription(msg_type, topic, lambda message: received.update({name: message}), qos)

    try:
        map_pub = node.create_publisher(OccupancyGrid, "/map", DURABLE)
        if args.multi:
            request_pub = node.create_publisher(FormationPlanRequest, "/formation/plan_request", 10)
            initial_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            subscribe("plan", FormationPlaybackPlan, "/formation/playback_plan")
            subscribe("path", Path, "/formation/path/vehicle_1")
            subscribe("markers", MarkerArray, "/formation/markers")
            subscribe("status", String, "/formation/optimizer_status")
        else:
            start_pub = node.create_publisher(PoseStamped, "/start_pose", 10)
            request_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
            subscribe("trajectory", TimedTrajectory, "/plan/timed_trajectory")
            subscribe("path", Path, "/plan/path")
            subscribe("markers", Marker, "/robot/marker", 10)
            subscribe("status", String, "/plan/status")
            subscribe("search", String, "/plan/search_status")
        assert spin_for(node, lambda: request_pub.get_subscription_count() > 0, 18.0), "planner subscription not discovered"
        assert spin_for(node, lambda: map_pub.get_subscription_count() >= 2, 18.0), "map subscribers not discovered"
        map_pub.publish(fixture_map(node, args.blocked))
        spin_for(node, lambda: False, 1.2)
        if args.multi:
            assert spin_for(node, lambda: initial_pub.get_subscription_count() >= 2, 8.0)
            initial = pose(node, START)
            reset = PoseWithCovarianceStamped(header=initial.header)
            reset.pose.pose = initial.pose
            initial_pub.publish(reset)
            spin_for(node, lambda: False, .6)
            request_pub.publish(formation_request(node))
            ready = lambda: ("plan" in received and received["plan"].trajectory.trajectories
                             and received.get("path") and received["path"].poses and "markers" in received)
        else:
            start_pub.publish(pose(node, START))
            spin_for(node, lambda: False, .4)
            request_pub.publish(pose(node, GOAL))
            ready = lambda: ("trajectory" in received and received["trajectory"].points
                             and received.get("path") and received["path"].poses and "markers" in received)
        status_message = lambda: getattr(received.get("status"), "data", "")[:800]
        if args.blocked:
            def rejection():
                if args.multi and "status" in received:
                    result = json.loads(received["status"].data)
                    if (result.get("node_state") == "REJECTED" and result.get("plan_id") == 600001
                            and not result.get("trajectory_published")):
                        return result["reason"]
                if not args.multi and "search" in received:
                    reason = received["search"].data.split()[0]
                    if reason in ("search_no_path_in_discretized_lattice", "search_budget_exhausted"):
                        return reason
                return None

            assert spin_for(node, rejection, args.timeout), status_message()
            assert not ready(), "a blocked goal produced a trajectory"
            print(f"blocked goal: {rejection()}")
            if args.json_out:
                args.json_out.write_text(json.dumps({"rejection": rejection()}) + "\n")
            return
        assert spin_for(node, ready, args.timeout), f"planner output timeout: {status_message()}"
        if args.multi:
            tracks = received["plan"].trajectory.trajectories
            assert [track.vehicle_id for track in tracks] == [1, 2, 3]
            for track, (dx, dy) in zip(tracks, OFFSETS):
                result = check_track(track.points, (START[0]+dx, START[1]+dy), (GOAL[0]+dx, GOAL[1]+dy))
                print(f"vehicle {track.vehicle_id}: {result[0]} points, {result[1]:.9f} s")
            sequences = [track.points for track in tracks]
            for poses in zip(*sequences):
                centers = [(entry.center_pose.x, entry.center_pose.y) for entry in poses]
                for i, j in [(0,1), (0,2), (1,2)]:
                    assert abs(math.dist(centers[i], centers[j])-math.dist(OFFSETS[i], OFFSETS[j])) < 1e-5
        else:
            sequences = [received["trajectory"].points]
            points, duration = check_track(sequences[0], START, GOAL)
            print(f"single vehicle: {points} points, {duration:.9f} s, path={len(received['path'].poses)}")
        if args.json_out:
            samples = [[[p.center_pose.x, p.center_pose.y, p.center_pose.theta,
                         int(p.mode), p.time_from_start.sec * 1_000_000_000 + p.time_from_start.nanosec,
                         p.center_speed_mps if args.multi else p.v]
                        for p in sequence] for sequence in sequences]
            args.json_out.write_text(json.dumps({"samples": samples}) + "\n")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
