#!/usr/bin/env python3
"""Send a scene experiment to a running planner and independently audit its plan."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from time import monotonic

from fwis_v3_msgs.msg import FormationPlanRequest, FormationPlaybackPlan, FormationVehicleSpec, TimedTrajectory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from ranger_mini_gz.scene_generator import SCENARIOS, TASKS, clearance_summary, load_scene
from ranger_mini_gz.scene_geometry import local_pose, rasterize


MAP_QOS = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wait(node, predicate, seconds):
    end = monotonic() + seconds
    while not predicate() and monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.10)
    return predicate()


def pose2d(value):
    return value.x, value.y, value.theta


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamped_pose(node, placement):
    x, y, yaw = placement
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x, message.pose.position.y = x, y
    message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return message


def request(node, scene, experiment):
    start, goal = experiment["start"], experiment["goal"]
    message = FormationPlanRequest(plan_id=710001, formation_id=scene.formation_id,
                                   allow_reverse=True)
    message.header = stamped_pose(node, start).header
    message.formation_start.x, message.formation_start.y, message.formation_start.theta = start
    message.formation_goal.x, message.formation_goal.y, message.formation_goal.theta = goal
    for vehicle, offset in zip(scene.vehicle_ids, scene.offsets):
        specification = FormationVehicleSpec(vehicle_id=vehicle, goal_heading_free=True)
        specification.offset_in_formation.x, specification.offset_in_formation.y = offset
        specification.start_center.x, specification.start_center.y, specification.start_center.theta = local_pose(
            start, offset)
        specification.goal_center.x, specification.goal_center.y, specification.goal_center.theta = local_pose(
            goal, offset)
        specification.body_length_m, specification.body_width_m = scene.vehicle_size[:2]
        message.vehicles.append(specification)
    return message


def result_rows(plan, scene, task):
    if task == "single":
        rows = [(pose2d(p.center_pose), None) for p in plan.points]
        return rows, seconds(plan.points[-1].time_from_start)
    trajectory = plan.trajectory
    tracks = {vehicle.vehicle_id: vehicle.points for vehicle in trajectory.trajectories}
    rows = [(pose2d(reference), [pose2d(tracks[index][i].center_pose) for index in scene.vehicle_ids])
            for i, reference in enumerate(trajectory.formation_poses)]
    return rows, seconds(trajectory.times_from_start[-1])


def run(node, args, scene):
    received = {}
    statuses = {}
    experiment = dict(scene.experiments[args.task])
    if args.goal is not None:
        experiment["goal"] = tuple(args.goal)
    spawns = scene.spawn_poses(args.task)
    node.create_subscription(OccupancyGrid, "/map", lambda msg: received.update(grid=msg), MAP_QOS)
    if not args.planner_only:
        for index in spawns:
            node.create_subscription(PoseStamped, f"/robot{index}/ground_truth_pose",
                                     lambda msg, i=index: received.update({i: msg}), 5)
    if args.task == "single":
        start_pub = node.create_publisher(PoseStamped, "/start_pose", 10)
        goal_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        plan_name, plan_type = "/plan/timed_trajectory", TimedTrajectory
        status_names = ("/plan/search_status", "/plan/status")

        def have_plan(msg):
            return bool(msg.points)

        def rejected():
            return (statuses.get(status_names[0], "").startswith(("search_no_path", "search_budget_exhausted",
                                                                  "action_policy_rejected")) or
                    statuses.get(status_names[1], "").startswith(("rejected ", "frontend_rejected ")))
    else:
        start_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        goal_pub = node.create_publisher(FormationPlanRequest, "/formation/plan_request", 10)
        plan_name, plan_type = "/formation/playback_plan", FormationPlaybackPlan
        status_names = ("/formation/optimizer_status",)
        node.create_subscription(String, "/formation/job_state",
                                 lambda msg: received.update(job=json.loads(msg.data)), MAP_QOS)

        def have_plan(msg):
            return msg.plan_id == 710001 and bool(msg.trajectory.formation_poses)

        def rejected():
            report = json.loads(statuses.get(status_names[0], "{}"))
            return report.get("plan_id") == 710001 and report.get("node_state") in ("FAILED", "REJECTED")

    def on_plan(message):
        if have_plan(message):
            received["plan"] = message

    node.create_subscription(plan_type, plan_name, on_plan, MAP_QOS)
    for topic in status_names:
        node.create_subscription(String, topic, lambda msg, key=topic: statuses.update({key: msg.data}), MAP_QOS)

    def start_subscriber_ready():
        return (start_pub.get_subscription_count()
                or received.get("job", {}).get("backend") == "uniform_v02")

    expected = (100 * rasterize(scene.obstacles, scene.size, scene.resolution)).astype(np.int8)
    if args.planner_only:
        map_pub = node.create_publisher(OccupancyGrid, "/map", MAP_QOS)
        if not wait(node, lambda: map_pub.get_subscription_count() >= 2
                   and start_subscriber_ready() and goal_pub.get_subscription_count(), 20):
            raise RuntimeError("the standalone planner subscribers did not arrive")
        grid = OccupancyGrid(header=stamped_pose(node, experiment["start"]).header)
        grid.info.resolution = scene.resolution
        grid.info.height, grid.info.width = expected.shape
        grid.info.origin.orientation.w = 1.0
        grid.data = expected.ravel().tolist()
        map_pub.publish(grid)
    if not wait(node, lambda: ("grid" in received
                               and (args.planner_only or all(i in received for i in spawns))
                               and start_subscriber_ready() and goal_pub.get_subscription_count()), 35):
        raise RuntimeError("map, Gazebo ground truth or planner subscribers did not arrive")
    message = received["grid"]
    cells = np.asarray(message.data, dtype=np.int8).reshape(message.info.height, message.info.width)
    assert message.header.frame_id == "map" and math.isclose(
        message.info.resolution, scene.resolution, rel_tol=1e-6)
    origin = message.info.origin
    assert (origin.position.x, origin.position.y, origin.orientation.z, origin.orientation.w) == (0, 0, 0, 1)
    assert np.array_equal(cells, expected), "received map differs from the scenario raster"
    if not args.planner_only:
        for index, (x, y, _yaw) in spawns.items():
            message = received[index]
            assert message.header.frame_id == "map"
            assert math.hypot(message.pose.position.x - x, message.pose.position.y - y) < .001
    initial = stamped_pose(node, experiment["start"])
    if args.task == "formation":
        reset = PoseWithCovarianceStamped(header=initial.header)
        reset.pose.pose = initial.pose
        start_pub.publish(reset)
    else:
        start_pub.publish(initial)
    wait(node, lambda: False, .4)
    begin = monotonic()
    goal_pub.publish(request(node, scene, experiment) if args.task == "formation"
                     else stamped_pose(node, experiment["goal"]))
    wait(node, lambda: "plan" in received or rejected(), args.timeout)
    elapsed = monotonic() - begin
    if args.task == "formation" and "plan" in received:
        wait(node, lambda: json.loads(statuses.get(status_names[0], "{}")).get(
            "trajectory_published", False), 1.0)
    report = {"scenario": scene.name, "variant": scene.variant, "task": args.task,
              "start": experiment["start"], "goal": experiment["goal"],
              "map_matches_generated": True, "gazebo_start_matches": None if args.planner_only else True,
              "occupancy_sha256": hashlib.sha256(cells.tobytes()).hexdigest(),
              "planner_wall_seconds": round(elapsed, 3), "status": statuses}
    if "plan" not in received:
        report["result"] = "rejected" if rejected() else "timeout"
        return report
    if args.planner_only and args.task == "formation":
        plan_id = received["plan"].plan_id

        def job_confirmed():
            job = received.get("job", {})
            return job.get("plan_id") == plan_id and job.get("state") in ("PLAYING", "HOLD_END")

        wait(node, lambda: job_confirmed() or rejected(), 35.0)
        report["playback_confirmed"] = job_confirmed()
        if job_confirmed():
            report["playback_confirmation_seconds"] = round(monotonic() - begin, 3)
    rows, duration = result_rows(received["plan"], scene, args.task)
    clearance = clearance_summary(scene, args.task, rows)
    start, goal = experiment["start"], experiment["goal"]
    report.update(
        result="trajectory_received",
        samples=len(rows),
        start_error_m=math.dist(rows[0][0][:2], start[:2]),
        goal_error_m=math.dist(rows[-1][0][:2], goal[:2]),
        goal_yaw_error_rad=abs(math.remainder(rows[-1][0][2] - goal[2], 2.0 * math.pi)),
        duration_s=duration,
        planar_length_m=sum(math.dist(a[0][:2], b[0][:2]) for a, b in zip(rows, rows[1:])),
        sampled_clearance=clearance,
        sampled_collision_free=all(value["distance_m"] > 0 for value in clearance.values()),
    )
    if args.task == "formation":
        envelope = received["plan"]
        report["planner_backend"] = envelope.backend
        report["planner_certification"] = envelope.certification
        mode_names = ("Ackermann", "Oblique", "Spin")
        report["vehicle_modes"] = {
            track.vehicle_id: sorted({mode_names[p.mode] for p in track.points})
            for track in envelope.trajectory.trajectories
        }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=SCENARIOS)
    parser.add_argument("task", choices=TASKS)
    parser.add_argument("--variant", default="default")
    parser.add_argument("--resolution", type=float)
    parser.add_argument("--goal", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    parser.add_argument("--timeout", type=float, default=55.)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--planner-only", action="store_true",
                        help="Publish the scene map and start to a standalone planner using wall time")
    args = parser.parse_args()
    if args.goal and not all(math.isfinite(value) for value in args.goal):
        parser.error("--goal must contain finite coordinates")
    scene = load_scene(args.scenario, args.variant, args.resolution)
    rclpy.init(args=[])
    node = rclpy.create_node("scene_plan_probe",
                            parameter_overrides=[Parameter("use_sim_time", value=not args.planner_only)])
    try:
        report = run(node, args, scene)
        content = json.dumps(report, ensure_ascii=False, indent=2)
        if args.output:
            args.output.write_text(content + "\n")
        print(content, flush=True)
        if args.planner_only and (report["result"] != "trajectory_received"
                                  or not report["sampled_collision_free"]
                                  or not report.get("playback_confirmed", True)):
            raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
