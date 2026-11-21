#!/usr/bin/env python3
"""Exercise B06/B10 on a private production planner, map and playback graph."""
import argparse
import math
import os
from pathlib import Path
import struct
import time

import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from fwis_v3_msgs.msg import FormationPlanRequest, FormationVehicleSpec
from fwis_v3_optimizer.formation_collision import FormationCollisionChecker
from fwis_v3_optimizer.formation_quality import grid_signature
from fwis_v3_optimizer.formation_request import validate_formation_request
from formation_uniform_planner_node import grid_from_occupancy_grid

from ros_uniform_playback_smoke import PROTECTED_PORTS, Session, write_json


def occupancy(block=None, *, malformed=False):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.info.origin.position.x = -4.0
    message.info.origin.position.y = -3.0
    message.info.origin.orientation.w = 1.0
    message.info.width = 0 if malformed else 200
    message.info.height = 150
    message.info.resolution = struct.unpack("<f", struct.pack("<f", 0.04))[0]
    message.data = [0] * (message.info.width * message.info.height)
    if block is not None:
        x, y, value = block
        col = math.floor((x + 4.0) / 0.04)
        row = math.floor((y + 3.0) / 0.04)
        message.data[row * message.info.width + col] = value
    return message


def request(plan_id, goal_xy, heading, free=False):
    message = FormationPlanRequest()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.plan_id = plan_id
    message.formation_id = rospy.get_param("/formation_playback/formation_id")
    message.allow_reverse = True
    message.formation_start = Pose2D(0.0, 0.0, 0.0)
    message.formation_goal = Pose2D(*goal_xy, 0.0)
    offsets = rospy.get_param("/formation_playback/offsets")
    for vehicle_id, (x, y) in zip(rospy.get_param("/formation_playback/vehicle_ids"), offsets):
        vehicle = FormationVehicleSpec()
        vehicle.vehicle_id = vehicle_id
        vehicle.offset_in_formation.x = x
        vehicle.offset_in_formation.y = y
        vehicle.body_length_m = rospy.get_param("/formation_playback/body_length_m")
        vehicle.body_width_m = rospy.get_param("/formation_playback/body_width_m")
        vehicle.start_center = Pose2D(x, y, 0.0)
        vehicle.goal_center = Pose2D(goal_xy[0] + x, goal_xy[1] + y, heading)
        vehicle.goal_heading_free = free
        message.vehicles.append(vehicle)
    return message


def exercise(session, report, screenshot_wait=0.0):
    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    request_pub = rospy.Publisher("/formation/plan_request", FormationPlanRequest, queue_size=1, latch=True)
    deadline = time.monotonic() + 20
    while map_pub.get_num_connections() < 1 or request_pub.get_num_connections() < 2:
        if time.monotonic() >= deadline:
            raise RuntimeError("map and request subscribers did not connect")
        rospy.sleep(0.05)

    def set_map(message):
        grid, _ = grid_from_occupancy_grid(message)
        key = None if grid is None else grid_signature(grid)
        map_pub.publish(message)
        session.wait_job(
            lambda job: job.get("map_signature") == key and job.get("map_ready") == (grid is not None),
            10, "selected occupancy grid")

    def run(message, expected=None):
        with session.lock:
            mark = len(session.playback)
        request_pub.publish(message)
        expected_state = "HOLD_END" if expected is None else "FAILED"
        job = session.wait_job(
            lambda document: document.get("plan_id") == message.plan_id
            and document.get("state") == expected_state, 15, expected_state)
        envelope = session.wait_envelope(message.plan_id, 5, executable=expected is None)
        if expected is None:
            assert envelope.model_json and envelope.model_sha256
            playback = session.wait_playback(
                lambda status: status.get("state") == "HOLD_END"
                and status.get("model_sha256") == job["model_sha256"],
                5, "valid terminal hold", mark=mark)
        else:
            assert not envelope.model_json and not envelope.model_sha256
            assert not envelope.trajectory.times_from_start
            playback = session.wait_playback(
                lambda status: status.get("state") == "CLEARED" and not status.get("executable"),
                5, "cleared receiver", mark=mark)
        deadline = rospy.Time.now().to_sec() + 5
        while rospy.Time.now().to_sec() < deadline:
            with session.lock:
                statuses = [doc for _, doc in session.statuses if doc.get("plan_id") == message.plan_id]
                status = statuses[-1] if statuses else None
            if status and status.get("job_state") == expected_state:
                break
            rospy.sleep(0.01)
        else:
            raise AssertionError("terminal optimizer status not received")
        if expected is not None:
            assert status["reason"] == expected and not status["executable"], status
            with session.lock:
                assert not any(env.model_json for _, env in session.envelope_records
                               if env.plan_id == message.plan_id)
        report[str(message.plan_id)] = {
            "job": job, "status": status, "playback_state": playback["state"],
            "empty_envelope": not bool(envelope.model_json)}
        if screenshot_wait:
            rospy.sleep(screenshot_wait)
        return status

    set_map(occupancy())
    run(request(600100, (0.0, 0.0), 0.0, free=True))

    spin_map = occupancy((1.0, 0.40, 100))
    set_map(spin_map)
    spin = request(600101, (0.0, 0.0), math.pi)
    snapshot = validate_formation_request(spin).snapshot
    grid, _ = grid_from_occupancy_grid(spin_map)
    checker = FormationCollisionChecker(grid, snapshot.model, snapshot.vehicle_shapes, 0.05)
    assert checker.static_check(snapshot.start, snapshot.start_headings).success
    assert checker.static_check(snapshot.goal, snapshot.goal_headings).success
    status = run(spin, "EXPANSION_BUDGET_EXHAUSTED")
    assert status["search"]["rejection_histogram"].get("MAP_OBB_COLLISION", 0) > 0
    report["600101"]["spin_endpoints_clear"] = True

    set_map(occupancy((1.0, 0.0, 100)))
    status = run(request(600102, (2.0, 0.0), 0.0), "MAP_OBB_COLLISION")
    assert "start static check" in status["detail"], status
    set_map(occupancy((3.0, 0.0, -1)))
    status = run(request(600103, (2.0, 0.0), 0.0), "MAP_OBB_COLLISION")
    assert "goal static check" in status["detail"], status
    set_map(occupancy(malformed=True))
    run(request(600104, (2.0, 0.0), 0.0), "MAP_UNAVAILABLE")
    set_map(occupancy())
    run(request(600105, (0.0, 0.0), 0.0, free=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    uri = os.environ.get("ROS_MASTER_URI", "")
    if not uri or int(uri.rstrip("/").rsplit(":", 1)[-1]) in PROTECTED_PORTS:
        parser.error("a private ROS master is required")
    rospy.init_node("uniform_safety_smoke", anonymous=True)
    session = Session()
    session.capture_display = False
    report = {}
    try:
        exercise(session, report)
        report["passed"] = True
    finally:
        write_json(args.output, report)


if __name__ == "__main__":
    main()
