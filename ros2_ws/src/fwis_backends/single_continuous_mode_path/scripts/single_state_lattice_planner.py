#!/usr/bin/env python3
"""Opt-in ROS adapter for the independently audited single-car state lattice."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, fields
import json
import math
from queue import Empty, SimpleQueue
import signal
from threading import Event
from time import monotonic

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from fwis_backends.single_continuous_mode_path.core import Action, Grid, Pose, wrap
from fwis_backends.single_continuous_mode_path.lattice_portfolio import optimize_legacy_seed
from fwis_backends.single_continuous_mode_path.mode_master import ModeMasterConfig
from fwis_backends.single_continuous_mode_path.quality import QualityProfile
from fwis_backends.single_continuous_mode_path.retiming import MotionLimits
from fwis_backends.single_continuous_mode_path.state_lattice import LatticeConfig, search_lattice
from fwis_v3_msgs.msg import PlanSeed, TimedPoint, TimedTrajectory


def planar_pose(pose):
    p, q = pose.position, pose.orientation
    if (not all(math.isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w))
            or max(abs(q.x), abs(q.y)) > 1e-7
            or abs(math.hypot(q.w, q.z) - 1.) > 1e-3):
        raise ValueError("expected a finite planar pose with unit quaternion")
    return Pose(p.x, p.y, math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z))


def matches_pose(pose, x, y, theta):
    return math.hypot(pose.x - x, pose.y - y) < 1e-7 and abs(wrap(pose.th - theta)) < 1e-7


@dataclass
class Contest:
    start: Pose
    goal: Pose
    grid: Grid
    limits: MotionLimits
    config: LatticeConfig
    deadline: float
    lattice: object = None
    legacy: object = None
    future: object = None
    start_sent: bool = False
    goal_stamp: int | None = None
    legacy_plan_id: int | None = None
    pending_seed: object = None
    legacy_finished: bool = False


class LatticePlanner(Node):
    def __init__(self):
        super().__init__("single_state_lattice_planner", automatically_declare_parameters_from_overrides=True)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.search_status = self.create_publisher(String, "/plan/search_status", durable)
        self.plan_status = self.create_publisher(String, "/plan/status", durable)
        self.trace = self.create_publisher(String, "/plan/lattice_trace", durable)
        self.trajectory = self.create_publisher(TimedTrajectory, "/plan/timed_trajectory", durable)
        self.path = self.create_publisher(Path, "/plan/path", durable)
        self.start = self.grid = self.map_id = self.job = None
        self.footprint = self.param("vehicle.length", .738), self.param("vehicle.width", .500)
        self.plan_id = 0
        self.generation = 0
        self.published = False
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="lattice")
        self.events = SimpleQueue()
        self.contest = self.request_context = self.job_cancel = None
        self.contest_distance = self.param("lattice.contest_distance_m", 6.)
        self.contest_window = self.param("lattice.contest_window_s", 14.)
        self.legacy_pool = None
        if self.param("lattice.contest_legacy", False):
            if (not all(math.isfinite(x) for x in (self.contest_distance, self.contest_window))
                    or self.contest_distance < 0. or self.contest_window <= 0.):
                raise ValueError("invalid lattice contest configuration")
            self.legacy_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="lattice_legacy")
            namespace = "/plan/lattice_legacy"
            self.legacy_ready = False
            self.legacy_start_pose = None
            self.legacy_start = self.create_publisher(PoseStamped, namespace + "/start", 1)
            self.legacy_goal = self.create_publisher(PoseStamped, namespace + "/goal", 1)
            self.create_subscription(String, namespace + "/search_status", self.on_legacy_status, durable)
            self.create_subscription(MarkerArray, namespace + "/markers", self.on_legacy_markers, durable)
            self.create_subscription(PlanSeed, namespace + "/request", self.on_legacy_request, durable)
            self.create_subscription(PlanSeed, namespace + "/seed", self.on_legacy_seed, durable)
        map_qos = durable if self.param("map_transient_local", True) else QoSProfile(depth=1)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(PoseStamped, "/start_pose", self.on_start, 1)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 1)
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.on_goal, 1)
        self.wakeup = self.create_timer(.01, self.on_timer)

    def param(self, name, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return default

    def identity(self):
        return Header(frame_id="map", stamp=self.get_clock().now().to_msg())

    def status(self, text):
        message = String(data=text)
        self.search_status.publish(message)
        self.plan_status.publish(message)

    def clear(self):
        self.trajectory.publish(TimedTrajectory(header=self.identity(), plan_id=self.plan_id))
        self.path.publish(Path(header=self.identity()))
        self.published = False

    def supersede(self):
        self.generation += 1
        self.discard_contest()
        if self.job_cancel is not None:
            self.job_cancel.set()
            self.job_cancel = None
        if self.job is not None:
            self.job.cancel()
            self.job = None

    def discard_contest(self):
        if self.contest is not None:
            if self.contest.future is not None:
                self.contest.future.cancel()
            self.contest = None
        self.request_context = None

    def on_map(self, message):
        info = message.info
        try:
            origin = planar_pose(info.origin)
            if (message.header.frame_id != "map" or abs(origin.th) > 1e-7
                    or info.width == 0 or info.height == 0
                    or not math.isfinite(info.resolution) or not info.resolution > 0.0):
                raise ValueError("invalid occupancy grid metadata")
            cells = np.asarray(message.data, dtype=np.int8).reshape(info.height, info.width)
        except ValueError as error:
            self.grid = self.map_id = None
            self.supersede()
            if self.published:
                self.clear()
            self.status(f"map_invalid reason={error}")
            return
        identity = info.width, info.height, info.resolution, origin, cells.tobytes()
        if identity != self.map_id:
            self.supersede()
            if self.published:
                self.clear()
            self.grid = Grid(cells != 0, info.resolution, origin.x, origin.y,
                             length=self.footprint[0], width=self.footprint[1],
                             margin=self.param("vehicle.safety_margin", .04))
            self.map_id = identity
        self.status(f"map_ready {info.width}x{info.height}")

    def on_start(self, message):
        self.set_start(message.header.frame_id, message.pose)

    def on_initialpose(self, message):
        self.set_start(message.header.frame_id, message.pose.pose)

    def set_start(self, frame, pose):
        try:
            if frame != "map":
                raise ValueError("start frame must be map")
            new_start = planar_pose(pose)
        except ValueError as error:
            new_start = None
            self.get_logger().warning(str(error))
        if new_start != self.start:
            self.supersede()
            if self.published:
                self.clear()
            self.start = new_start

    def limits(self):
        limits = MotionLimits(profile="ranger_state_lattice_research", v_max=.16,
                              a_max=.22, jerk_max=.8, omega_max=.4, alpha_max=.45,
                              angular_jerk_max=1., r_min_ack=.9, mode_switch_duration=.7)
        overrides = {field.name: self.param(f"quality.limits.{field.name}", getattr(limits, field.name))
                     for field in fields(limits)}
        for name in ("r_min_ack", "wheelbase", "track", "mode_setup_time", "mode_switch_duration"):
            overrides[name] = self.param(f"vehicle.{name}", overrides[name])
        return MotionLimits(**overrides)

    def on_goal(self, message):
        self.supersede()
        self.plan_id = self.plan_id % 0xffffffff + 1
        self.clear()
        if self.grid is None or self.start is None:
            self.status(f"missing_map_or_start id={self.plan_id}")
            return
        if message.header.frame_id != "map":
            self.status(f"invalid_goal_frame id={self.plan_id}")
            return
        try:
            goal = planar_pose(message.pose)
            limits = self.limits()
            config = LatticeConfig(
                resolution_m=self.param("lattice.resolution_m", .3),
                terminal_radius_m=self.param("lattice.terminal_radius_m", 2.0),
                max_expansions=self.param("lattice.max_expansions", 5000),
                time_budget_s=self.param("lattice.time_budget_s", 2.0),
                allow_reverse=self.param("lattice.allow_reverse", True),
                refine_continuous=self.param("lattice.refine_continuous", False),
                analytic_ackermann=self.param("lattice.analytic_ackermann", True))
        except ValueError as error:
            self.status(f"rejected id={self.plan_id} reason={error}")
            return
        if self.legacy_pool is not None:
            self.request_context = Contest(self.start, goal, self.grid, limits, config,
                                   monotonic() + self.contest_window)
            if math.hypot(goal.x - self.start.x, goal.y - self.start.y) >= self.contest_distance:
                self.contest = self.request_context
        self.job_cancel = Event()
        plan_id, generation = self.plan_id, self.generation
        self.status(f"search_pending id={plan_id}")
        self.job = self.pool.submit(search_lattice, self.start, goal, self.grid, limits, config,
                                    lambda candidate: self.events.put((plan_id, generation, candidate)),
                                    footprint_m=self.footprint, cancelled=self.job_cancel.is_set)

    def publish_trace(self, **data):
        self.trace.publish(String(data=json.dumps({"plan_id": self.plan_id, **data}, allow_nan=False)))

    def on_legacy_status(self, message):
        if message.data.startswith("map_ready "):
            self.legacy_ready = True

    def on_legacy_markers(self, message):
        for marker in message.markers:
            if marker.ns == "v3_endpoints" and marker.id == 0 and marker.action == Marker.ADD:
                try:
                    self.legacy_start_pose = planar_pose(marker.pose)
                except ValueError:
                    self.legacy_start_pose = None

    def matches_legacy_request(self, message):
        contest = self.contest
        if (contest is None or contest.goal_stamp is None or contest.legacy_finished
                or message.header.frame_id != "map" or message.plan_id == 0):
            return False
        stamp = message.header.stamp
        return (
            stamp.sec * 1_000_000_000 + stamp.nanosec >= contest.goal_stamp
            and matches_pose(contest.start, message.start_center.x, message.start_center.y,
                             message.start_center.theta)
            and matches_pose(contest.goal, message.goal_center.x, message.goal_center.y,
                             message.goal_center.theta))

    def on_legacy_request(self, message):
        if not self.matches_legacy_request(message):
            return
        contest = self.contest
        if contest.legacy_plan_id is None:
            contest.legacy_plan_id = message.plan_id
            if contest.pending_seed is not None:
                pending, contest.pending_seed = contest.pending_seed, None
                self.on_legacy_seed(pending)

    def on_legacy_seed(self, message):
        if not message.primitives or not self.matches_legacy_request(message):
            return
        contest = self.contest
        if contest.legacy_plan_id is None:
            contest.pending_seed = message
            return
        if contest.future is not None or message.plan_id != contest.legacy_plan_id:
            return
        seed = tuple(Action(action.mode, action.amount, action.control) for action in message.primitives)
        if any(action.mode not in (0, 1, 2) or not all(math.isfinite(x) for x in
                                                       (action.amount, action.control)) for action in seed):
            return
        master = ModeMasterConfig()
        settings = {field.name: self.param(f"quality.search.{field.name}", getattr(master, field.name))
                    for field in fields(master)}
        settings.update(continuous_motion_v2=True, allow_reverse=contest.config.allow_reverse,
                        allow_shunting=False, selection_objective="minimum_time",
                        mode_anchor_extra_budget_s=0.)
        settings["budget_s"] = min(settings["budget_s"], contest.deadline - monotonic() - .10)
        if settings["budget_s"] <= settings["validation_reserve_s"] + settings["candidate_evaluation_reserve_s"]:
            contest.legacy_finished = True
            return
        defaults = QualityProfile()
        profile = QualityProfile(**{item.name: self.param(f"quality.cost.{item.name}",
                                                        getattr(defaults, item.name))
                                    for item in fields(defaults)})
        try:
            config = ModeMasterConfig(**settings)
        except ValueError as error:
            self.get_logger().warning(f"Invalid legacy contest configuration: {error}")
            contest.legacy_finished = True
            return
        contest.future = self.legacy_pool.submit(
            optimize_legacy_seed, seed, contest.start, contest.goal, contest.grid, contest.limits,
            self.footprint, config, profile, analytic_ackermann=contest.config.analytic_ackermann)

    def pose_message(self, pose):
        message = PoseStamped(header=self.identity())
        message.pose.position.x, message.pose.position.y = pose.x, pose.y
        message.pose.orientation.z = math.sin(pose.th / 2.)
        message.pose.orientation.w = math.cos(pose.th / 2.)
        return message

    def tick_contest(self):
        contest = self.contest
        in_time = monotonic() < contest.deadline
        if in_time and self.legacy_ready and self.legacy_start.get_subscription_count() > 0:
            if not contest.start_sent:
                self.legacy_start_pose = None
                self.legacy_start.publish(self.pose_message(contest.start))
                contest.start_sent = True
            elif (contest.goal_stamp is None and self.legacy_start_pose is not None
                  and matches_pose(contest.start, self.legacy_start_pose.x,
                                   self.legacy_start_pose.y, self.legacy_start_pose.th)
                  and self.legacy_goal.get_subscription_count() > 0):
                message = self.pose_message(contest.goal)
                stamp = message.header.stamp
                contest.goal_stamp = stamp.sec * 1_000_000_000 + stamp.nanosec
                self.legacy_goal.publish(message)
        if in_time and contest.future is not None and contest.future.done() and not contest.legacy_finished:
            contest.legacy_finished = True
            try:
                contest.legacy = contest.future.result()
            except (ValueError, ArithmeticError, RuntimeError) as error:
                self.get_logger().warning(f"Legacy seed {self.plan_id} rejected: {error}")
        if monotonic() < contest.deadline and (self.job is not None or not contest.legacy_finished):
            return
        candidates = [(kind, candidate) for kind, candidate in
                      (("state_lattice", contest.lattice), ("continuous_portfolio", contest.legacy))
                      if candidate is not None]
        if candidates:
            source, chosen = min(candidates, key=lambda item: item[1].timed.duration)
            self.publish_certified(self.plan_id, chosen, source=source)
            self.publish_trace(
                phase="portfolio_selected", source=source,
                candidates_s={kind: candidate.timed.duration for kind, candidate in candidates},
                min_safe_clearance_m={
                    kind: candidate.quality["metrics"]["min_safe_clearance_m"]
                    for kind, candidate in candidates})
        else:
            self.status(f"search_no_path id={self.plan_id} reason=NO_CERTIFIED_PORTFOLIO_CANDIDATE")
        self.supersede()

    def on_timer(self):
        while True:
            try:
                plan_id, generation, candidate = self.events.get_nowait()
            except Empty:
                break
            if plan_id == self.plan_id and generation == self.generation and not self.published:
                self.offer_lattice_candidate(plan_id, candidate)
        if self.job is not None and self.job.done():
            job, self.job = self.job, None
            try:
                result = job.result()
            except (ValueError, ArithmeticError, RuntimeError) as error:
                if self.contest is None and not self.published:
                    self.status(f"search_no_path id={self.plan_id} reason={error}")
            else:
                self.report_lattice_result(result)
        if self.contest is not None:
            self.tick_contest()

    def offer_lattice_candidate(self, plan_id, candidate):
        modes = {action.mode for action in candidate.actions}
        if (self.contest is None and self.request_context is not None
                and (len(modes) > 1 or len(candidate.actions) > 3)):
            self.contest = self.request_context
        if self.contest is not None:
            self.contest.lattice = candidate
        else:
            self.publish_certified(plan_id, candidate)

    def report_lattice_result(self, result):
        self.publish_trace(
            first_certified_s=result.first_certified_s, wall_s=result.wall_s, reason=result.reason,
            phase="search_complete", already_published=self.published, expansions=result.expanded,
            tested_goals=result.tested_goals, rejected_goals=result.rejected_goals,
            best_certified_duration_s=result.incumbent.timed.duration if result.incumbent else None)
        if self.contest is not None:
            self.contest.lattice = result.incumbent
        elif not self.published:
            if result.incumbent is not None:
                self.offer_lattice_candidate(self.plan_id, result.incumbent)
            else:
                self.status(f"search_no_path id={self.plan_id} reason={result.reason}")

    def publish_certified(self, plan_id, candidate, *, source="state_lattice"):
        if self.published or candidate.quality["status"] != "FEASIBLE":
            return
        header = self.identity()
        path = Path(header=header)
        message = TimedTrajectory(header=header, plan_id=plan_id)
        for time_s, pose, speed, mode in candidate.timed.samples:
            nanos = round(time_s * 1e9)
            secs, remainder = divmod(nanos, 10 ** 9)
            point = TimedPoint(time_from_start=Duration(sec=secs, nanosec=remainder), v=speed, mode=mode)
            point.center_pose.x, point.center_pose.y, point.center_pose.theta = pose.x, pose.y, pose.th
            message.points.append(point)
            vertex = PoseStamped(header=header)
            vertex.pose.position.x, vertex.pose.position.y = pose.x, pose.y
            vertex.pose.orientation.z = math.sin(pose.th / 2.0)
            vertex.pose.orientation.w = math.cos(pose.th / 2.0)
            path.poses.append(vertex)
        self.path.publish(path)
        self.trajectory.publish(message)
        self.published = True
        self.status(f"success id={plan_id} source={source} duration={candidate.timed.duration:.6f}")
        self.get_logger().info(f"Published independently certified lattice plan {plan_id}")


def main():
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    # Finish the current callback before shutting down the executor.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    planner = LatticePlanner()
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(planner, timeout_sec=.1)
    except ExternalShutdownException:
        pass
    finally:
        planner.supersede()
        planner.pool.shutdown(wait=True, cancel_futures=True)
        if planner.legacy_pool is not None:
            planner.legacy_pool.shutdown(wait=True, cancel_futures=True)
        planner.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
