#!/usr/bin/env python3
"""Record read-only DDS delivery timestamps for one CopTrans test run."""

import argparse
import json
import re
from pathlib import Path
from time import monotonic_ns

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from fwis_v3_msgs.msg import (FormationExecutionPermit, FormationPlanRequest,
                              FormationPlaybackPlan, TimedTrajectory)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Observer(Node):
    def __init__(self, run_dir, task, output):
        super().__init__("coptrans_baseline_observer")
        self.run = run_dir.name
        self.output = output
        self.seen = set()
        self.plan_id = None
        self.pending_commands = {}
        durable = QoSProfile(depth=2, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if task == "single":
            self.create_subscription(PoseStamped, "/move_base_simple/goal", self.single_request, 10)
            self.create_subscription(String, "/plan/search_status", self.single_search, durable)
            self.create_subscription(TimedTrajectory, "/plan/timed_trajectory", self.single_trajectory, durable)
        else:
            self.create_subscription(FormationPlanRequest, "/formation/plan_request", self.request, 10)
            self.create_subscription(FormationPlaybackPlan, "/formation/playback_plan", self.formation_plan, durable)
            self.create_subscription(String, "/formation/optimizer_status", self.planner_status, durable)
            self.create_subscription(String, "/formation/job_state", self.job_state, durable)
            self.create_subscription(FormationExecutionPermit, "/formation/execution_permit", self.permit, 10)
        for robot in range(1, 2 if task == "single" else 4):
            prefix = f"/robot{robot}"
            self.create_subscription(String, f"{prefix}/controller_status",
                                     lambda message, index=robot: self.control_status(message, index), 10)
            self.create_subscription(Twist, f"{prefix}/cmd_vel",
                                     lambda message, index=robot: self.command(message, index), 10)
            if task == "multi":
                self.create_subscription(TimedTrajectory, f"{prefix}/trajectory",
                                         lambda message, index=robot: self.route(message, index), durable)
        self.mark("observer_ready")

    def mark(self, event, plan_id=None, vehicle_id=None, *, observed_ns=None, **data):
        key = (event, plan_id, vehicle_id, data.get("phase"))
        if key in self.seen:
            return
        self.seen.add(key)
        row = {
            "trace_id": self.run,
            "event": event,
            "plan_id": plan_id,
            "vehicle_id": vehicle_id,
            "observed_monotonic_ns": monotonic_ns() if observed_ns is None else observed_ns,
            **data,
        }
        self.output.write(json.dumps(row, allow_nan=False) + "\n")
        self.output.flush()

    def single_request(self, message):
        self.mark("request_received", ros_stamp_ns=stamp_ns(message.header.stamp))

    def request(self, message):
        self.mark("request_received", message.plan_id, ros_stamp_ns=stamp_ns(message.header.stamp))

    def single_search(self, message):
        match = re.match(r"(search_\w+) id=(\d+)\b", message.data)
        if match:
            self.mark(match[1], int(match[2]))

    def single_trajectory(self, message):
        if message.points:
            self.plan_id = message.plan_id
            self.mark("trajectory_received", message.plan_id, ros_stamp_ns=stamp_ns(message.header.stamp),
                      point_count=len(message.points))
            self.flush_command(1)

    def formation_plan(self, message):
        if message.trajectory.formation_poses:
            self.plan_id = message.plan_id
            self.mark("trajectory_received", message.plan_id, ros_stamp_ns=stamp_ns(message.header.stamp),
                      point_count=len(message.trajectory.formation_poses), model_sha256=message.model_sha256)
            for robot in self.pending_commands.copy():
                self.flush_command(robot)

    def planner_status(self, message):
        status = json.loads(message.data)
        plan_id = status.get("plan_id")
        if plan_id and status.get("admitted"):
            self.mark("planner_request_admitted", plan_id)
        if plan_id and status.get("trajectory_published"):
            self.mark("planner_document_reports_published", plan_id,
                      planning_wall_s=status.get("wall_time_s"))

    def job_state(self, message):
        status = json.loads(message.data)
        if status.get("plan_id"):
            self.mark("planner_job_state", status["plan_id"], phase=status.get("state"),
                      model_sha256=status.get("model_sha256"))

    def permit(self, message):
        if message.enabled:
            self.mark("permit_received", message.plan_id, start_ros_ns=stamp_ns(message.start))

    def route(self, message, robot):
        if message.points:
            self.mark("route_received", message.plan_id, robot,
                      start_ros_ns=stamp_ns(message.header.stamp), point_count=len(message.points))

    def control_status(self, message, robot):
        status = json.loads(message.data)
        plan_id = status.get("plan_id")
        if not plan_id:
            return
        if status.get("kind") == "admission" and status.get("result") != "CLEARED":
            self.mark("controller_admission_received", plan_id, robot, phase=status.get("phase"),
                      result=status.get("result"), sim_s=status.get("stamp"),
                      plan_start_ros_ns=status.get("trajectory_stamp_ns"))
        elif status.get("kind") == "execution" and status.get("phase") in ("TRACKING", "COMPLETE"):
            self.mark("controller_phase_received", plan_id, robot, phase=status["phase"],
                      sim_s=status.get("stamp"))

    def command(self, message, robot):
        command = [message.linear.x, message.linear.y, message.angular.z]
        if any(abs(component) > 1e-12 for component in command):
            if self.plan_id is not None:
                self.mark("first_nonzero_twist_received", self.plan_id, robot, twist=command)
            else:
                self.pending_commands.setdefault(robot, (monotonic_ns(), command))

    def flush_command(self, robot):
        pending = self.pending_commands.pop(robot, None)
        if pending is not None:
            timestamp, command = pending
            self.mark("first_nonzero_twist_received", self.plan_id, robot, observed_ns=timestamp,
                      twist=command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("task", choices=("single", "multi"))
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rclpy.init()
    with args.output.open("w", encoding="utf-8") as output:
        observer = Observer(args.run_dir, args.task, output)
        try:
            rclpy.spin(observer)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        finally:
            observer.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
