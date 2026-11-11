#!/usr/bin/env python3
"""T3 real-entry smoke: E00-E04 on the empty map, NLP off or on.

One invocation runs one NLP configuration through the real user entry point,
``run_demo.sh``, on a private ROS master.  Every attempt uses the real input
path: ``2D Pose Estimate`` (``/initialpose``) resets the cargo to
``(10, 10, 0)`` with all body headings zero, then ``2D Nav Goal`` publishes
the mission goal and the goal bridge builds the request.

The five T3 missions are exercised ``--attempts`` times each (default 3):

* E00 start == goal: ``NO_MOTION`` audited static hold, one t=0 sample;
* E01 straight +x: Ackermann, positive signed speed;
* E02 lateral +y: oblique, body headings stay zero (not forced to the travel
  direction);
* E03 oblique (+3, +2): oblique at ``atan2(2, 3)``, positive signed speed;
* E04 reverse -x: Ackermann, negative signed speed.

Each successful attempt is checked against the optimizer status, the timed
trajectory, the independent quality selection, playback ``HOLD_END``, the
marker OBBs and the playback TF.  Failures are recorded per attempt and never
deleted.  Run the script twice, once per NLP configuration::

    python3 tests/ros_t3_smoke.py --enable-nlp false \
        --output evaluation/wedge_fix_2026-11-11/T3/smoke_nlp_off.json \
        --log-dir evaluation/wedge_fix_2026-11-11/T3/runtime_nlp_off
    python3 tests/ros_t3_smoke.py --enable-nlp true --port 11448 \
        --output evaluation/wedge_fix_2026-11-11/T3/smoke_nlp_on.json \
        --log-dir evaluation/wedge_fix_2026-11-11/T3/runtime_nlp_on

The private master defaults are outside the protected interactive ports
11311/11312/11320.  The script only stops the processes it started.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

import yaml

ROOT = Path(__file__).resolve().parents[1]
TESTS = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "tools"))
from ros_formation_smoke import (  # noqa: E402
    prepare_python_path,
    sourced_environment,
)
sys.path.insert(0, str(TESTS))
from ros_cargo_smoke import (  # noqa: E402
    CONFIG,
    EMPTY_MAP,
    PROTECTED_PORTS,
    Observations,
    SmokeError,
    check,
    close,
    close_angle,
    expected_centre,
    make_goal,
    port_open,
    same_pose,
    send_initial,
    utc_now,
    wait_for,
    wait_port,
    wait_port_closed,
    write_json,
)

START = (10.0, 10.0, 0.0)
POSITION_TOL = 0.02          # T3 terminal baseline: 0.02 m
HEADING_TOL = math.radians(1.0)  # T3 terminal baseline: 1 degree
RIGIDITY_TOL = 1.0e-6
MARKER_TOL = 1.0e-4
TF_TOL = 1.0e-3
MISSIONS = (
    ("E00", (10.0, 10.0, 0.0), "NO_MOTION"),
    ("E01", (13.0, 10.0, 0.0), "FEASIBLE"),
    ("E02", (10.0, 13.0, 0.0), "FEASIBLE"),
    ("E03", (13.0, 12.0, 0.0), "FEASIBLE"),
    ("E04", (7.0, 10.0, 0.0), "FEASIBLE"),
)
FINAL_STATUSES = ("FEASIBLE", "NO_MOTION", "INVALID", "REJECTED",
                  "GEOMETRY_ONLY_NOT_EXECUTABLE")


class T3Observations(Observations):
    def __init__(self):
        super().__init__()
        self.displays = []
        self.seeds = []

    def add_display(self, message):
        self.displays.append((time.time(), message))
        del self.displays[:-50]

    def add_seed(self, message):
        self.seeds.append((time.time(), message))
        del self.seeds[:-50]

    def seed_for(self, plan_id, after):
        for wall, message in self.seeds:
            if wall >= after and int(message.plan_id) == plan_id:
                return wall, message
        return None

    def display_at(self, after, pose):
        for wall, message in reversed(self.displays):
            if wall < after:
                continue
            value = (float(message.pose.position.x),
                     float(message.pose.position.y),
                     math.atan2(2.0 * message.pose.orientation.w *
                                message.pose.orientation.z,
                                1.0 - 2.0 * message.pose.orientation.z ** 2))
            if same_pose(value, pose, 1.0e-6, 1.0e-6):
                return wall, value
        return None

    def request_for(self, after, goal):
        for wall, request in self.requests:
            if wall < after:
                continue
            if (same_pose(request["formation_start"], START, 1.0e-6, 1.0e-6)
                    and same_pose(request["formation_goal"], goal,
                                  1.0e-6, 1.0e-6)
                    and all(close(item["start_center"][2], 0.0, 1.0e-6)
                            for item in request["vehicles"])):
                return wall, request
        return None

    def final_status(self, plan_id, after, expected):
        for wall, document in self.optimizer:
            if wall < after or document.get("plan_id") != plan_id:
                continue
            status = document.get("status")
            if status == expected and (
                    expected == "NO_MOTION"
                    or document.get("trajectory_published") is True):
                return wall, document
            if status in ("INVALID", "REJECTED",
                          "GEOMETRY_ONLY_NOT_EXECUTABLE"):
                return wall, document
        return None

    def trajectory_for(self, plan_id, after):
        for wall, message in self.trajectories:
            if (wall >= after and int(message.plan_id) == plan_id
                    and len(message.times_from_start) > 0
                    and len(message.trajectories) >= 3):
                return wall, message
        return None

    def quality_for(self, plan_id, after, expected, published=False):
        for wall, document in self.quality:
            if wall < after or document.get("plan_id") != plan_id:
                continue
            if document.get("status") != expected:
                continue
            if published and document.get("trajectory_published") is not True:
                continue
            return wall, document
        return None

    def hold_end(self, plan_id, after):
        for wall, document in self.playback:
            if (wall >= after and document.get("plan_id") == plan_id
                    and document.get("state") == "HOLD_END"
                    and document.get("terminal") is True):
                return wall, document
        return None

    def playback_executable(self, plan_id, after, expected):
        for wall, document in self.playback:
            if (wall >= after and document.get("plan_id") == plan_id
                    and document.get("state") == "HOLD_END"
                    and document.get("terminal") is True
                    and document.get("executable") is True
                    and document.get("certification") == expected):
                return wall, document
        return None

    def markers_with_bodies(self, after):
        for wall, array in reversed(self.markers):
            if wall < after:
                continue
            bodies = sum(marker.type == 4 and marker.action == 0
                         and "vehicle_" in marker.ns
                         for marker in array.markers)
            if bodies >= 3:
                return wall, array
        return None

    def markers_at_terminal(self, after, formation_id, vehicle_ids,
                            centres):
        item = self.markers_with_bodies(after)
        if item is None:
            return None
        wall, array = item
        try:
            bodies = body_geometry(array, formation_id, vehicle_ids)
        except SmokeError:
            return None
        for vehicle_id in vehicle_ids:
            target = centres[vehicle_id]
            centre = bodies[vehicle_id]["centre"]
            if math.hypot(centre[0] - target[0],
                          centre[1] - target[1]) > MARKER_TOL:
                return None
        return wall, array

    def fresh_tf(self, after):
        pending = [child for child in self.tf
                   if self.tf[child]["received"] >= after]
        if len(pending) != 4:
            return None
        return {child: dict(self.tf[child]) for child in pending}


def final_status_of(terminal):
    return terminal.get("status")


def require(result, name, ok, detail):
    ok = bool(ok)
    result["checks"][name] = {"ok": ok, "detail": str(detail)}
    if not ok:
        raise SmokeError("{}: {}".format(name, detail))


def timed_values(message):
    return [value.to_sec() for value in message.times_from_start]


def point_values(point):
    return {
        "time": point.time_from_start.to_sec(),
        "center": (float(point.center_pose.x), float(point.center_pose.y)),
        "theta": float(point.center_pose.theta),
        "speed": float(point.center_speed_mps),
        "yaw_rate": float(point.center_yaw_rate_radps),
        "mode": int(point.mode),
        "control": float(point.mode_control),
    }


def body_geometry(array, formation_id, vehicle_ids):
    """Extract the OBB centre/heading from a marker array.

    Playback publishes the OBB, the heading arrow, a mode band and a text
    marker in each vehicle namespace.  Select by geometry instead of assuming
    the T1 marker count.
    """
    found = {}
    for vehicle_id in vehicle_ids:
        namespace = "formation/{}/vehicle_{}".format(
            formation_id, vehicle_id)
        markers = [marker for marker in array.markers
                   if marker.ns == namespace and marker.action == 0]
        strips = [marker for marker in markers if marker.type == 4
                  and len(marker.points) == 5]
        arrows = [marker for marker in markers if marker.type == 0
                  and len(marker.points) == 2]
        if len(strips) != 1 or len(arrows) != 1:
            raise SmokeError(
                "vehicle {} marker set is not one closed OBB + one arrow: "
                "{}".format(vehicle_id,
                            [(marker.type, len(marker.points))
                             for marker in markers]))
        corners = [(float(point.x), float(point.y))
                   for point in strips[0].points]
        centre = (sum(point[0] for point in corners[:4]) / 4.0,
                  sum(point[1] for point in corners[:4]) / 4.0)
        arrow = arrows[0].points
        heading = math.atan2(arrow[1].y - arrow[0].y,
                             arrow[1].x - arrow[0].x)
        found[vehicle_id] = {"centre": centre, "heading": heading}
    return found


def seed_snapshot(message):
    return {
        "plan_id": int(message.plan_id),
        "primitives": [
            {"type": int(item.geometry_type),
             "dx": float(item.dx), "dy": float(item.dy),
             "dpsi": float(item.dpsi),
             "pivot": [float(item.pivot_x), float(item.pivot_y)]}
            for item in message.primitives],
    }


def trajectory_snapshot(message):
    points = {int(trajectory.vehicle_id):
              [point_values(point) for point in trajectory.points]
              for trajectory in message.trajectories}
    summary = {
        "plan_id": int(message.plan_id),
        "duration_s": (message.times_from_start[-1].to_sec()
                       if message.times_from_start else None),
        "sample_count": len(message.times_from_start),
        "vehicles": {},
    }
    for vehicle_id, samples in points.items():
        moving = [point for point in samples
                  if abs(point["speed"]) > 1.0e-6 or point["mode"] == 1]
        summary["vehicles"][vehicle_id] = {
            "mode_runs": [],
            "speed_min": min((point["speed"] for point in moving), default=0.0),
            "speed_max": max((point["speed"] for point in moving), default=0.0),
            "theta_min": min((point["theta"] for point in sampling(samples)),
                             default=0.0),
            "theta_max": max((point["theta"] for point in sampling(samples)),
                             default=0.0),
        }
        for point in moving:
            key = (point["mode"], round(point["control"], 6))
            if (not summary["vehicles"][vehicle_id]["mode_runs"]
                    or summary["vehicles"][vehicle_id]["mode_runs"][-1][0]
                    != key):
                summary["vehicles"][vehicle_id]["mode_runs"].append(
                    [key, 1])
            else:
                summary["vehicles"][vehicle_id]["mode_runs"][-1][1] += 1
    return summary


def sampling(samples):
    """Evenly sample a trajectory for compact evidence."""
    if len(samples) <= 32:
        return samples
    step = max(1, len(samples) // 31)
    return samples[::step] + [samples[-1]]


def mission_geometry_checks(result, mission, message, config):
    times = timed_values(message)
    require(result, "trajectory_starts_at_zero",
            bool(times) and abs(times[0]) <= 1.0e-9, times[:3])
    if len(times) > 1:
        require(result, "trajectory_time_strict",
                all(second - first > 1.0e-9
                    for first, second in zip(times, times[1:])),
                "samples={} last={}".format(len(times), times[-1]))
    else:
        require(result, "trajectory_single_sample", mission == "E00",
                "{} samples for {}".format(len(times), mission))
    offsets = {
        int(item.vehicle_id): (float(item.offset_in_formation.x),
                               float(item.offset_in_formation.y))
        for item in message.vehicles}
    require(result, "trajectory_vehicle_identity",
            sorted(offsets) == sorted(config["vehicle_ids"])
            and all(math.hypot(offsets[vehicle_id][0] - offset[0],
                               offsets[vehicle_id][1] - offset[1]) <= 1.0e-9
                    for vehicle_id, offset in zip(
                        config["vehicle_ids"], config["offsets"])),
            offsets)
    poses = [(float(pose.x), float(pose.y), float(pose.theta))
             for pose in message.formation_poses]
    require(result, "trajectory_pose_count", len(poses) == len(times),
            "poses={} times={}".format(len(poses), len(times)))
    require(result, "trajectory_cargo_terminal",
            same_pose(poses[-1], MISSIONS_BY_ID[mission][0],
                      POSITION_TOL, HEADING_TOL),
            "terminal={} goal={}".format(poses[-1],
                                         MISSIONS_BY_ID[mission][0]))
    points_by_vehicle = {}
    for trajectory in message.trajectories:
        vehicle_id = int(trajectory.vehicle_id)
        require(result, "trajectory_unique_vehicle_{}".format(vehicle_id),
                vehicle_id not in points_by_vehicle, vehicle_id)
        points = [point_values(point) for point in trajectory.points]
        require(result, "trajectory_point_count_{}".format(vehicle_id),
                len(points) == len(times),
                "{} != {}".format(len(points), len(times)))
        for index, point in enumerate(points):
            require(result, "trajectory_time_match_{}_{}".format(
                vehicle_id, index),
                abs(point["time"] - times[index]) <= 1.0e-9,
                (point["time"], times[index]))
        for index, (pose, point) in enumerate(zip(poses, points)):
            expected = expected_centre(pose, offsets[vehicle_id])
            error = math.hypot(point["center"][0] - expected[0],
                               point["center"][1] - expected[1])
            require(result, "trajectory_rigidity_{}_{}".format(
                vehicle_id, index), error <= RIGIDITY_TOL,
                "error={:.3e}".format(error))
        points_by_vehicle[vehicle_id] = points
    require(result, "trajectory_all_vehicles",
            sorted(points_by_vehicle) == sorted(config["vehicle_ids"]),
            sorted(points_by_vehicle))

    for vehicle_id, points in points_by_vehicle.items():
        moving = [point for point in points
                  if abs(point["speed"]) > 1.0e-6
                  or point["mode"] == 1]
        if mission == "E00":
            require(result, "e00_one_point_{}".format(vehicle_id),
                    len(points) == 1, len(points))
            point = points[0]
            require(result, "e00_static_{}".format(vehicle_id),
                    point["speed"] == 0.0 and point["yaw_rate"] == 0.0,
                    point)
            require(result, "e00_control_{}".format(vehicle_id),
                    point["mode"] == 0 and abs(point["control"]) <= 1.0e-12,
                    (point["mode"], point["control"]))
            require(result, "e00_heading_{}".format(vehicle_id),
                    close_angle(point["theta"], 0.0, 1.0e-9),
                    point["theta"])
            continue
        require(result, mission + "_moving_{}".format(vehicle_id),
                bool(moving), "no moving sample")
        require(result, mission + "_heading_{}".format(vehicle_id),
                all(close_angle(point["theta"], 0.0, 1.0e-6)
                    for point in moving),
                sorted({round(point["theta"], 9) for point in moving}))
        if mission == "E01":
            require(result, "e01_modes_{}".format(vehicle_id),
                    all(point["mode"] == 0 for point in moving),
                    sorted({point["mode"] for point in moving}))
            require(result, "e01_speed_positive_{}".format(vehicle_id),
                    all(point["speed"] > 0.0 for point in moving),
                    (min(point["speed"] for point in moving),
                     max(point["speed"] for point in moving)))
        elif mission == "E02":
            require(result, "e02_oblique_{}".format(vehicle_id),
                    all(point["mode"] == 1 and
                        abs(abs(point["control"]) - math.pi / 2.0) <= 1.0e-6
                        for point in moving),
                    sorted({(point["mode"], round(point["control"], 6))
                            for point in moving}))
            require(result, "e02_not_body_forward_{}".format(vehicle_id),
                    all(abs(point["speed"]) <= 1.0e-6 for point in moving),
                    (min(point["speed"] for point in moving),
                     max(point["speed"] for point in moving)))
            delta_y = points[-1]["center"][1] - points[0]["center"][1]
            require(result, "e02_lateral_displacement_{}".format(vehicle_id),
                    delta_y > 2.9, delta_y)
        elif mission == "E03":
            require(result, "e03_oblique_{}".format(vehicle_id),
                    all(point["mode"] == 1 for point in moving),
                    sorted({(point["mode"], round(point["control"], 6))
                            for point in moving}))
            driving = [point for point in moving
                       if abs(point["speed"]) > 1.0e-6]
            require(result, "e03_speed_positive_{}".format(vehicle_id),
                    bool(driving) and all(point["speed"] > 0.0
                                          for point in driving),
                    (min(point["speed"] for point in driving),
                     max(point["speed"] for point in driving)))
            delta_x = points[-1]["center"][0] - points[0]["center"][0]
            delta_y = points[-1]["center"][1] - points[0]["center"][1]
            require(result, "e03_displacement_{}".format(vehicle_id),
                    abs(delta_x - 3.0) <= 1.0e-6
                    and abs(delta_y - 2.0) <= 1.0e-6,
                    (delta_x, delta_y))
        elif mission == "E04":
            require(result, "e04_modes_{}".format(vehicle_id),
                    all(point["mode"] == 0 for point in moving),
                    sorted({point["mode"] for point in moving}))
            require(result, "e04_speed_negative_{}".format(vehicle_id),
                    all(point["speed"] < 0.0 for point in moving),
                    (min(point["speed"] for point in moving),
                     max(point["speed"] for point in moving)))
            delta_x = points[-1]["center"][0] - points[0]["center"][0]
            require(result, "e04_reverse_displacement_{}".format(vehicle_id),
                    delta_x < -2.9, delta_x)


MISSIONS_BY_ID = {label: (goal, status) for label, goal, status in MISSIONS}


def run_attempt(mission, attempt_index, publishers, observations, process,
                config):
    goal, expected_status = MISSIONS_BY_ID[mission]
    result = {
        "mission": mission,
        "attempt": attempt_index,
        "goal": list(goal),
        "expected_status": expected_status,
        "plan_id": None,
        "checks": {},
        "ok": False,
        "error": None,
    }
    rospy = publishers["rospy"]
    try:
        reset_wall = time.time()
        send_initial(publishers["initial"], *START)
        wait_for(
            lambda: observations.display_at(reset_wall, START),
            mission + " attempt {} start preview".format(attempt_index),
            10.0, process, observations)
        goal_wall = time.time()
        publishers["goal"].publish(make_goal(*goal))
        request_wall, request = wait_for(
            lambda: observations.request_for(goal_wall, goal),
            mission + " attempt {} bridge request".format(attempt_index),
            15.0, process, observations)
        plan_id = int(request["plan_id"])
        result["plan_id"] = plan_id
        require(result, "request_start_headings",
                all(close(item["start_center"][2], 0.0, 1.0e-6)
                    for item in request["vehicles"]), "not all zero")
        terminal_wall, terminal = wait_for(
            lambda: observations.final_status(plan_id, goal_wall,
                                              expected_status),
            mission + " attempt {} terminal status".format(attempt_index),
            120.0, process, observations)
        result["terminal_status"] = terminal
        require(result, "terminal_status",
                final_status_of(terminal) == expected_status,
                "status={} reason={}".format(
                    terminal.get("status"), terminal.get("reason")))
        require(result, "terminal_plan_id", terminal.get("plan_id") == plan_id,
                (terminal.get("plan_id"), plan_id))
        require(result, "terminal_feasible",
                terminal.get("feasible") is True, terminal.get("feasible"))
        require(result, "terminal_executable",
                terminal.get("executable") is True,
                terminal.get("executable"))
        quality_wall, quality = wait_for(
            lambda: observations.quality_for(plan_id, goal_wall,
                                             expected_status,
                                             published=True),
            mission + " attempt {} quality report".format(attempt_index),
            30.0, process, observations)
        result["quality_report"] = {
            "status": quality.get("status"),
            "reason": quality.get("reason"),
            "independent_audit": quality.get("independent_audit"),
            "collision_certificate": quality.get("collision_certificate"),
            "pipeline_status": (quality.get("pipeline") or {}).get("status"),
            "T_end": (quality.get("pipeline") or {}).get("T_end"),
            "no_motion": (quality.get("pipeline") or {}).get("no_motion"),
        }
        require(result, "quality_independent_audit",
                quality.get("independent_audit") is True,
                quality.get("independent_audit"))
        require(result, "quality_collision_certificate",
                quality.get("collision_certificate") is True,
                quality.get("collision_certificate"))
        require(result, "quality_pipeline_status",
                (quality.get("pipeline") or {}).get("status")
                == expected_status,
                (quality.get("pipeline") or {}).get("status"))
        if expected_status == "NO_MOTION":
            require(result, "no_motion_flag",
                    terminal.get("no_motion") is True,
                    terminal.get("no_motion"))
            require(result, "no_motion_contract",
                    bool(terminal.get("hold_contract")),
                    terminal.get("hold_contract"))
            require(result, "no_motion_reason",
                    terminal.get("reason") == "NO_MOTION",
                    terminal.get("reason"))
        else:
            require(result, "feasible_no_motion_flag",
                    terminal.get("no_motion") is False,
                    terminal.get("no_motion"))

        trajectory_wall, trajectory = wait_for(
            lambda: observations.trajectory_for(plan_id, goal_wall),
            mission + " attempt {} timed trajectory".format(attempt_index),
            15.0, process, observations)
        result["trajectory"] = trajectory_snapshot(trajectory)
        seed = observations.seed_for(plan_id, goal_wall)
        if seed is not None:
            result["seed"] = seed_snapshot(seed[1])
        require(result, "trajectory_identity",
                trajectory.header.frame_id == "map"
                and trajectory.formation_id == config["formation_id"]
                and int(trajectory.plan_id) == plan_id,
                (trajectory.header.frame_id, trajectory.formation_id,
                 trajectory.plan_id))
        times = timed_values(trajectory)
        duration = times[-1] if times else 0.0
        mission_geometry_checks(result, mission, trajectory, config)

        first_wall, first_hold = wait_for(
            lambda: observations.hold_end(plan_id, goal_wall),
            mission + " attempt {} playback HOLD_END".format(attempt_index),
            duration + 25.0, process, observations)
        result["playback_first_terminal"] = {
            "wall": first_wall,
            "executable": first_hold.get("executable"),
            "certification": first_hold.get("certification"),
        }
        hold_wall, hold = wait_for(
            lambda: observations.playback_executable(
                plan_id, goal_wall, expected_status),
            mission + " attempt {} audited playback".format(attempt_index),
            20.0, process, observations)
        result["playback_status_upgrade_s"] = round(hold_wall - first_wall, 3)
        result["playback_terminal"] = hold
        require(result, "playback_plan_id", hold.get("plan_id") == plan_id,
                hold.get("plan_id"))
        require(result, "playback_terminal", hold.get("terminal") is True,
                hold.get("terminal"))
        require(result, "playback_executable", hold.get("executable") is True,
                hold.get("executable"))
        require(result, "playback_certification",
                hold.get("certification") == expected_status,
                hold.get("certification"))
        require(result, "playback_duration",
                abs(float(hold.get("duration_s", -1.0)) - duration) <= 1.0e-9,
                (hold.get("duration_s"), duration))
        require(result, "playback_sample_count",
                int(hold.get("sample_count", -1))
                == (1 if mission == "E00" else len(times)),
                (hold.get("sample_count"), len(times)))
        vehicles = {int(item["vehicle_id"]): item
                    for item in hold.get("vehicles", [])}
        require(result, "playback_vehicle_ids",
                sorted(vehicles) == sorted(config["vehicle_ids"]),
                sorted(vehicles))
        for vehicle_id, offset in zip(config["vehicle_ids"],
                                      config["offsets"]):
            item = vehicles[vehicle_id]
            centre = expected_centre(goal, offset)
            require(result, "playback_center_{}".format(vehicle_id),
                    math.hypot(item["center"][0] - centre[0],
                               item["center"][1] - centre[1])
                    <= POSITION_TOL, (item["center"], centre))
            require(result, "playback_heading_{}".format(vehicle_id),
                    close_angle(item["theta"], 0.0, HEADING_TOL),
                    item["theta"])

        expected_centres = {
            vehicle_id: expected_centre(goal, offset)
            for vehicle_id, offset in zip(config["vehicle_ids"],
                                          config["offsets"])}
        marker_wall, array = wait_for(
            lambda: observations.markers_at_terminal(
                hold_wall, config["formation_id"], config["vehicle_ids"],
                expected_centres),
            mission + " attempt {} terminal markers".format(attempt_index),
            15.0, process, observations)
        bodies = body_geometry(array, config["formation_id"],
                               config["vehicle_ids"])
        for vehicle_id, offset in zip(config["vehicle_ids"],
                                      config["offsets"]):
            centre = expected_centre(goal, offset)
            require(result, "marker_center_{}".format(vehicle_id),
                    math.hypot(bodies[vehicle_id]["centre"][0] - centre[0],
                               bodies[vehicle_id]["centre"][1] - centre[1])
                    <= MARKER_TOL,
                    (bodies[vehicle_id]["centre"], centre))
            require(result, "marker_heading_{}".format(vehicle_id),
                    close_angle(bodies[vehicle_id]["heading"], 0.0, 1.0e-4),
                    bodies[vehicle_id]["heading"])
        status_text = next(
            (marker.text for marker in array.markers
             if marker.ns == "formation/{}/status".format(
                 config["formation_id"]) and marker.type == 9), "")
        require(result, "marker_status_label",
                expected_status in status_text, status_text[:200])

        frames = wait_for(
            lambda: observations.fresh_tf(marker_wall),
            mission + " attempt {} TF".format(attempt_index),
            15.0, process, observations)
        reference = frames["formation_{}_reference".format(
            config["formation_id"])]
        require(result, "tf_reference_pose",
                same_pose((reference["translation"][0],
                           reference["translation"][1], reference["yaw"]),
                          goal, TF_TOL, TF_TOL),
                (reference["translation"], reference["yaw"]))
        for vehicle_id, offset in zip(config["vehicle_ids"],
                                      config["offsets"]):
            child = "formation_{}_vehicle_{}_base_link".format(
                config["formation_id"], vehicle_id)
            require(result, "tf_vehicle_{}_present".format(vehicle_id),
                    child in frames, sorted(frames))
            centre = expected_centre(goal, offset)
            item = frames[child]
            require(result, "tf_center_{}".format(vehicle_id),
                    math.hypot(item["translation"][0] - centre[0],
                               item["translation"][1] - centre[1])
                    <= TF_TOL,
                    (item["translation"], centre))
            require(result, "tf_heading_{}".format(vehicle_id),
                    close_angle(item["yaw"], 0.0, TF_TOL), item["yaw"])
        result["ok"] = True
    except SmokeError as exc:
        result["error"] = str(exc)
    except Exception as exc:  # noqa: BLE001 - recorded as evidence
        result["error"] = "{}: {}".format(type(exc).__name__, exc)
        result["traceback"] = traceback.format_exc()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=11447)
    parser.add_argument("--enable-nlp", choices=("true", "false"),
                        default="false")
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument(
        "--output",
        default="evaluation/wedge_fix_2026-11-11/T3/smoke.json")
    parser.add_argument(
        "--log-dir", default="evaluation/wedge_fix_2026-11-11/T3/runtime")
    args = parser.parse_args(argv)
    if args.attempts <= 0:
        parser.error("--attempts must be positive")
    started = time.time()
    enabled = args.enable_nlp == "true"
    output = Path(args.output).resolve()
    log_dir = Path(args.log_dir).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    summary = {
        "schema": "ros_t3_smoke/v1",
        "started_at": utc_now(),
        "finished_at": None,
        "duration_s": None,
        "pass": False,
        "port": args.port,
        "enable_nlp": enabled,
        "attempts": args.attempts,
        "command": [
            "bash", "run_demo.sh", "--map", str(EMPTY_MAP),
            "--scale", "1.0", "auto_goal:=false", "rviz:=false",
            "enable_nlp:={}".format(args.enable_nlp)],
        "config": {
            "path": str(CONFIG),
            "sha256": hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
            "formation_id": config["formation_id"],
            "vehicle_ids": config["vehicle_ids"],
            "offsets": config["offsets"],
            "body_length_m": config["body_length_m"],
            "body_width_m": config["body_width_m"],
            "allow_reverse": config["allow_reverse"],
            "goal_heading_free": config["goal_heading_free"],
            "initial_heading_policy": config["initial_heading_policy"],
        },
        "checks": {},
        "missions": {
            label: {"attempts": [], "success_count": 0}
            for label, _, _ in MISSIONS},
        "observed": {},
        "errors": [],
        "cleanup": {},
        "logs": {},
    }
    run_demo = None
    master = None
    master_log = None
    demo_log = None
    rospy = None
    observations = None
    exit_codes = {}
    try:
        if args.port in PROTECTED_PORTS:
            raise SmokeError(
                "port {} is reserved for the interactive demo".format(
                    args.port))
        if port_open(args.port):
            raise SmokeError(
                "private master port {} is already in use".format(args.port))
        if not EMPTY_MAP.is_dir():
            raise SmokeError("empty map missing: {}".format(EMPTY_MAP))
        environment = sourced_environment()
        environment.update({
            "ROS_MASTER_URI": "http://127.0.0.1:{}".format(args.port),
            "ROS_HOSTNAME": "127.0.0.1",
            "ROS_HOME": str(log_dir / "ros_home"),
            "ROS_LOG_DIR": str(log_dir / "ros_logs"),
            "PYTHONUNBUFFERED": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
        })
        os.environ.update({
            "ROS_MASTER_URI": environment["ROS_MASTER_URI"],
            "ROS_HOME": environment["ROS_HOME"],
            "ROS_LOG_DIR": environment["ROS_LOG_DIR"],
        })
        prepare_python_path(environment)

        import rospy as imported_rospy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from visualization_msgs.msg import MarkerArray
        from fwis_v3_msgs.msg import (
            FormationPlanRequest, FormationPlanSeed,
            FormationTimedTrajectory)
        rospy = imported_rospy

        master_log = (log_dir / "roscore.log").open("w")
        master = subprocess.Popen(
            ["roscore", "-p", str(args.port)], env=environment,
            stdout=master_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        wait_port(args.port, 20.0, "private roscore")
        demo_log = (log_dir / "run_demo.log").open("w")
        demo_command = [
            "bash", str(ROOT / "run_demo.sh"), "--map", str(EMPTY_MAP),
            "--scale", "1.0", "auto_goal:=false", "rviz:=false",
            "enable_nlp:={}".format(args.enable_nlp)]
        run_demo = subprocess.Popen(
            demo_command, cwd=str(ROOT), env=environment,
            stdout=demo_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        check(summary, "real_entry_point_started",
              run_demo.poll() is None,
              "run_demo.sh pid={}".format(run_demo.pid))

        rospy.init_node("t3_smoke", anonymous=True, disable_signals=True)
        observations = T3Observations()
        rospy.Subscriber("/formation/plan_request", FormationPlanRequest,
                         observations.add_request, queue_size=20)
        rospy.Subscriber("/formation/optimizer_status", String,
                         observations.add_optimizer, queue_size=100)
        rospy.Subscriber("/formation/quality_report", String,
                         observations.add_quality, queue_size=50)
        rospy.Subscriber("/formation/playback_status", String,
                         observations.add_playback, queue_size=200)
        rospy.Subscriber("/formation/markers", MarkerArray,
                         observations.add_markers, queue_size=100)
        rospy.Subscriber("/formation/timed_trajectory",
                         FormationTimedTrajectory,
                         observations.add_trajectory, queue_size=20)
        rospy.Subscriber("/formation/plan_seed", FormationPlanSeed,
                         observations.add_seed, queue_size=20)
        rospy.Subscriber("/formation/display_pose", PoseStamped,
                         observations.add_display, queue_size=20)
        rospy.Subscriber("/tf", TFMessage, observations.add_tf,
                         queue_size=100)
        initial_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)
        goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1)
        request_pub = rospy.Publisher(
            "/formation/plan_request", FormationPlanRequest, queue_size=1)
        wait_for(
            lambda: initial_pub.get_num_connections() > 0
            and goal_pub.get_num_connections() > 0,
            "goal bridge input subscribers", 120.0, run_demo, observations)
        wait_for(
            lambda: request_pub.get_num_connections() >= 4,
            "formation plan_request subscribers ready", 30.0, run_demo,
            observations)
        parameter = wait_for(
            lambda: ((True, rospy.get_param(
                "/formation_optimizer/enable_nlp"))
                     if rospy.has_param("/formation_optimizer/enable_nlp")
                     else None),
            "optimizer enable_nlp parameter", 30.0, run_demo, observations)
        check(summary, "enable_nlp_effective", bool(parameter[1]) == enabled,
              "parameter={} requested={}".format(parameter[1], enabled))
        publishers = {"rospy": rospy, "initial": initial_pub,
                      "goal": goal_pub}
        for mission, _, _ in MISSIONS:
            for attempt_index in range(1, args.attempts + 1):
                result = run_attempt(
                    mission, attempt_index, publishers, observations,
                    run_demo, config)
                summary["missions"][mission]["attempts"].append(result)
                summary["observed"].setdefault(mission, []).append({
                    "attempt": attempt_index,
                    "plan_id": result["plan_id"],
                    "ok": result["ok"],
                    "error": result["error"],
                    "terminal_status": (
                        None if "terminal_status" not in result
                        else {
                            "status": result["terminal_status"].get("status"),
                            "reason": result["terminal_status"].get("reason"),
                            "duration_s": result["terminal_status"].get(
                                "duration_s"),
                        }),
                })
                print("  [{}] {} attempt {} plan={} {}".format(
                    "ok" if result["ok"] else "FAIL", mission,
                    attempt_index, result["plan_id"],
                    result["error"] or ""))
        for label in summary["missions"]:
            attempts = summary["missions"][label]["attempts"]
            success = sum(1 for item in attempts if item["ok"])
            summary["missions"][label]["success_count"] = success
            summary["missions"][label]["success_rate"] = (
                float(success) / float(len(attempts)) if attempts else 0.0)
            check(summary, label + "_all_attempts_succeed",
                  success == args.attempts,
                  "{}/{} attempts succeeded".format(success, len(attempts)))
        summary["pass"] = True
    except Exception as exc:  # noqa: BLE001 - recorded as evidence
        summary["errors"].append({
            "type": type(exc).__name__,
            "detail": str(exc),
        })
        if observations is not None:
            summary["observed"]["failure_snapshot"] = observations.brief()
    finally:
        if rospy is not None:
            try:
                rospy.signal_shutdown("t3 smoke complete")
            except Exception:
                pass
        for process, label in ((run_demo, "run_demo"), (master, "roscore")):
            if process is None:
                continue
            exit_codes[label] = None
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    pass
            exit_codes[label] = process.poll()
        for handle in (demo_log, master_log):
            if handle is not None:
                handle.flush()
                handle.close()
        released = wait_port_closed(args.port, 10.0)
        summary["cleanup"] = {
            "exit_codes": exit_codes,
            "port_released": released,
        }
        summary["logs"] = {
            "run_demo": str(log_dir / "run_demo.log"),
            "roscore": str(log_dir / "roscore.log"),
            "ros_logs": str(log_dir / "ros_logs"),
        }
    summary["finished_at"] = utc_now()
    summary["duration_s"] = round(time.time() - started, 3)
    if not summary["checks"]:
        summary["pass"] = False
    write_json(output, summary)
    print("ros_t3_smoke[enable_nlp={}]: {} ({} checks, {})".format(
        args.enable_nlp, "PASS" if summary["pass"] else "FAIL",
        len(summary["checks"]), output))
    for name, item in summary["checks"].items():
        print("  [{}] {}: {}".format(
            "ok" if item["ok"] else "FAIL", name, item["detail"][:300]))
    if summary["errors"]:
        print("errors:", file=sys.stderr)
        for error in summary["errors"]:
            print("  {}: {}".format(error["type"], error["detail"]),
                  file=sys.stderr)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())