#!/usr/bin/env python3
"""Paired ROS-only latency benchmark for the installed single-car planner."""
import argparse
from contextlib import ExitStack
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import tempfile
from time import monotonic

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from fwis_v3_msgs.msg import TimedTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
import yaml

from ranger_mini_gz.scene_generator import load_scene
from ranger_mini_gz.scene_geometry import rasterize


ROOT = Path(__file__).resolve().parents[2]
BACKEND = "single_continuous_mode_path"
SCENARIO = "factory_indoor_01"
MODE_NAMES = ("A", "O", "S")
DURABLE = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)
FLAGS = {
    "baseline": {"adaptive_reverse_competition": False, "seed_proposal_wait_s": .15},
    "optimized": {"adaptive_reverse_competition": True, "seed_proposal_wait_s": 0.},
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def merge(base, *overrides):
    target = deepcopy(base)
    for layer in overrides:
        for key, value in layer.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                target[key] = merge(target[key], value)
            else:
                target[key] = deepcopy(value)
    return target


def config():
    resolver = subprocess.check_output([
        str(ROOT / "scripts/test.sh"), "--task", "single",
        "--architecture-profile", "baseline", "--speed-profile", "baseline",
        "--scenario", SCENARIO, "--experiment", "short_turn", "--dry-run"], cwd=ROOT, text=True)
    return yaml.safe_load(resolver)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def labeled(raw):
    labels = {}
    for key, value in re.findall(r"(\w+)=([^ ]+)", raw):
        try:
            labels[key] = float(value) if "." in value else int(value)
        except ValueError:
            labels[key] = value
    return labels


def trajectory_quality(message, config):
    digest = hashlib.sha256()
    modes, coordinates, times = [], [], []
    for p in message.points:
        time_ns = p.time_from_start.sec * 10**9 + p.time_from_start.nanosec
        pose = p.center_pose
        digest.update(struct.pack("<q4dB", time_ns, pose.x, pose.y, pose.theta, p.v, p.mode))
        if not modes or modes[-1] != MODE_NAMES[p.mode]:
            modes.append(MODE_NAMES[p.mode])
        coordinates.append((pose.x, pose.y, pose.theta))
        times.append(time_ns)
    goal = config["goal"]
    return {
        "trajectory_content_sha256": digest.hexdigest(),
        "samples": len(coordinates), "mode_sequence": modes,
        "duration_s": times[-1] / 1e9,
        "strictly_increasing_times": all(a < b for a, b in zip(times, times[1:])),
        "terminal_position_error_m": math.dist(coordinates[-1][:2], goal[:2]),
        "terminal_yaw_error_rad": abs(math.remainder(coordinates[-1][2] - goal[2], math.tau)),
        "max_uploaded_translation_speed_mps": max(abs(p.v) for p in message.points),
    }


class PlannerCase:
    def __init__(self, label, domain, binaries, data, params, directory, executor_stack):
        self.label, self.count = label, 0
        self.plan = self.quality = None
        self.messages = {"search": "", "optimizer": ""}
        self.latency = None
        params_path = directory / f"{label}.yaml"
        params_path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}, sort_keys=False))
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
                   ROS_LOG_DIR=str(directory / f"{label}-ros"), OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
        self.processes = []
        for kind, executable in binaries.items():
            log = executor_stack.enter_context((directory / f"{label}-{kind}.log").open("w"))
            process = subprocess.Popen(
                [str(executable), "--ros-args", "--params-file", str(params_path)],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            executor_stack.callback(stop, process)
            self.processes.append(process)

        context = Context()
        rclpy.init(context=context, args=[], domain_id=domain)
        executor_stack.callback(context.try_shutdown)
        self.node = rclpy.create_node(f"bench_single_{label}", context=context)
        executor_stack.callback(self.node.destroy_node)
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        executor_stack.callback(self.executor.shutdown)
        self.map = self.node.create_publisher(OccupancyGrid, "/map", DURABLE)
        self.start = self.node.create_publisher(PoseStamped, "/start_pose", 10)
        self.goal = self.node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        self.node.create_subscription(TimedTrajectory, "/plan/timed_trajectory", self.on_plan, DURABLE)
        self.node.create_subscription(String, "/plan/quality_report", self.on_quality, DURABLE)
        for key, topic in (("search", "/plan/search_status"), ("optimizer", "/plan/status")):
            self.node.create_subscription(String, topic, lambda msg, k=key: self.on_status(k, msg), DURABLE)
        self.data = data

    def until(self, predicate, timeout):
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            if predicate():
                return True
            if any(p.poll() is not None for p in self.processes):
                raise RuntimeError(f"{self.label} planner subprocess exited")
            self.executor.spin_once(timeout_sec=min(.1, max(0., deadline-monotonic())))
        return predicate()

    def pose(self, location):
        x, y, yaw = location
        message = PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.orientation.w = math.cos(yaw * .5)
        message.pose.orientation.z = math.sin(yaw * .5)
        return message

    def ready(self, cells, resolution):
        if not self.until(lambda: self.map.get_subscription_count() == 2 and
                          self.start.get_subscription_count() >= 1 and
                          self.goal.get_subscription_count() >= 1, 20.):
            raise RuntimeError(f"{self.label} planner endpoints were not discovered")
        grid = OccupancyGrid(header=self.pose(self.data["start"]).header)
        grid.info.resolution = resolution
        grid.info.height, grid.info.width = cells.shape
        grid.info.origin.orientation.w = 1.
        grid.data = cells.ravel().tolist()
        self.map.publish(grid)
        if not self.until(lambda: all(v.startswith("map_ready ") for v in self.messages.values()), 20.):
            raise RuntimeError(f"{self.label} map readiness timed out: {self.messages}")
        self.start.publish(self.pose(self.data["start"]))
        self.until(lambda: False, .5)

    def on_plan(self, message):
        if message.points and message.plan_id == self.count and self.plan is None:
            self.latency = monotonic() - self.started
            self.plan = message

    def on_quality(self, message):
        parsed = json.loads(message.data)
        if parsed.get("plan_id") == self.count and parsed.get("status") != "pending":
            self.quality = parsed

    def on_status(self, key, message):
        value = message.data
        if not self.count or labeled(value).get("id") == self.count:
            self.messages[key] = value

    def rejected(self):
        return self.messages["search"].startswith((
            "search_no_path", "search_budget_exhausted", "action_policy_rejected", "missing_map_or_start")) or self.messages["optimizer"].startswith((
            "rejected ", "frontend_rejected ", "geometry_only ", "reverse_coarse_seed_rejected "))

    def trial(self, timeout):
        self.count += 1
        self.plan = self.quality = self.latency = None
        self.messages = {"search": "", "optimizer": ""}
        goal = self.pose(self.data["goal"])
        self.started = monotonic()
        self.goal.publish(goal)
        self.until(lambda: self.plan is not None or self.rejected(), timeout)
        elapsed = monotonic() - self.started
        if self.plan is not None:
            self.until(lambda: self.quality is not None and self.messages["optimizer"].startswith("success "), 4.)
        elif self.rejected():
            self.until(lambda: self.quality is not None, 1.)
        quality = self.quality or {}
        selected = quality.get("selected", {})
        metrics = selected.get("metrics", {})
        row = {
            "plan_id": self.count, "wall_s": self.latency if self.latency is not None else elapsed,
            "result": "trajectory_received" if self.plan is not None else "rejected" if self.rejected() else "timeout",
            "front_end_status": self.messages["search"],
            "optimizer_status": self.messages["optimizer"],
            "front_end_fields": labeled(self.messages["search"]),
            "independent_audit": {key: selected.get(key) for key in (
                "status", "geometry_status", "timing_status", "dynamic_status", "score")},
            "quality_status": quality.get("status"),
            "execution_display_sweep_clear": quality.get("execution_display_sweep_clear"),
            "trajectory_quality": {key: metrics.get(key) for key in (
                "length_m", "min_body_clearance_m", "min_safe_clearance_m", "mode_switches", "direction_switches")},
            "optimizer_compute_s": quality.get("optimizer_callback_wall_s"),
            "optimizer_stages_s": quality.get("stage_timings_s"),
        }
        if self.plan is not None:
            row.update(trajectory_quality(self.plan, self.data))
        return row


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def summarize(rows):
    good = [row for row in rows if row["result"] == "trajectory_received"]
    values = [row["wall_s"] for row in good]
    return {
        "samples": len(rows), "successes": len(good), "failures": len(rows) - len(good),
        "wall_s": {name: percentile(values, value) for name, value in
                   (("p50", .5), ("p95", .95), ("max", 1.))} if values else {},
        "trajectory_hashes": sorted({row["trajectory_content_sha256"] for row in good}),
        "mode_sequences": sorted({tuple(row["mode_sequence"]) for row in good}),
    }


def write(output, data):
    temp = output.with_suffix(".tmp")
    temp.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temp.replace(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=24.)
    parser.add_argument("--base-domain", type=int, default=101)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    base = config()
    overrides = base["planner_overrides"]
    assert overrides["quality"]["limits"]["target_v"] == .16
    installed = Path(get_package_prefix("fwis_backends")).resolve()
    package_path = installed / "lib" / "fwis_backends"
    binaries = {role: package_path / f"{BACKEND}_{role}" for role in ("planner", "optimizer")}
    assert b"search.adaptive_reverse_competition" in binaries["planner"].read_bytes()
    assert b"self.seed_proposal_wait_s" in binaries["optimizer"].read_bytes()
    vehicle = yaml.safe_load((Path(get_package_share_directory("fwis_common")) / "config/ranger_mini_2.yaml").read_text())
    defaults = yaml.safe_load((Path(get_package_share_directory("fwis_backends")) /
                               "config" / BACKEND / "params.yaml").read_text())
    scene = load_scene(SCENARIO)
    cells = (100 * rasterize(scene.obstacles, scene.size, scene.resolution)).astype(np.int8)
    report = {
        "scenario": SCENARIO, "experiment": "short_turn", "start": base["start"], "goal": base["goal"],
        "planner_limits": overrides, "git_sha": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "installed_package_prefix": str(installed),
        "binary_sha256": {role: sha256(exe) for role, exe in binaries.items()},
        "source_state": "working tree integration build, check hashes",
        "map_sha256": hashlib.sha256(cells.tobytes()).hexdigest(),
        "flags": FLAGS, "requested_warmups": args.warmups, "requested_trials": args.trials,
        "layout": "alternating paired requests, one outstanding at a time, separate ROS DDS domains",
        "clock": "monotonic wall time from goal publish to first nonempty trajectory receipt; no Gazebo or RViz",
        "warmups": {key: [] for key in FLAGS}, "trials": {key: [] for key in FLAGS}, "status": "running",
    }
    with tempfile.TemporaryDirectory(prefix="coptrans-single-ros-") as temporary, ExitStack() as stack:
        directory = Path(temporary)
        cases = {}
        for index, (label, flag) in enumerate(FLAGS.items()):
            params = merge(vehicle, defaults, overrides, {
                "search": {"adaptive_reverse_competition": flag["adaptive_reverse_competition"]},
                "quality": {"seed_proposal_wait_s": flag["seed_proposal_wait_s"],
                            "report_dir": str(directory / f"{label}-plans")},
                "use_sim_time": False, "map_transient_local": True,
            })
            cases[label] = PlannerCase(label, args.base_domain + index, binaries, base, params, directory, stack)
        for case in cases.values():
            case.ready(cells, scene.resolution)
        write(args.output, report)
        for block, count in (("warmups", args.warmups), ("trials", args.trials)):
            for i in range(count):
                order = tuple(cases) if i % 2 == 0 else tuple(reversed(cases))
                for label in order:
                    row = cases[label].trial(args.timeout)
                    row.update(pair=i + 1, order=list(order))
                    report[block][label].append(row)
                    report["summary"] = {key: summarize(rows) for key, rows in report["trials"].items()}
                    write(args.output, report)
                    print(f"{block} {i+1}/{count} {label} {row['result']} {row['wall_s']:.3f} s", flush=True)
    report["status"] = "complete"
    write(args.output, report)


if __name__ == "__main__":
    main()
