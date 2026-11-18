#!/usr/bin/env python3
"""V2 real ROS scene driver: one actual corridor plan through planner+playback.

This is test/evidence tooling only.  It publishes the synthetic narrow-curve
scene (an OccupancyGrid), the RViz 2D Pose Estimate and one ordinary
FormationPlanRequest.  From that point on every displayed byte comes from the
real formation_uniform_planner_node (A* route, C2-A chain/guide and its own
planning_debug document) and the real formation_playback node (corridor,
guide and final-path MarkerArrays).  No plan, chain, guide or marker is
fabricated here.

At the narrow-curve midpoint the driver records the latest display payloads,
captures the RViz window and writes the canonical payload hash.  It also
independently rebuilds the expected virtual request from the raw request and
the planner's job record and calls the production envelope verifier, then
compares the live playback_status frame with frame_at()'s authoritative
VirtualATimedPlan state at the same elapsed time.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf.transformations import quaternion_from_euler

from fwis_v3_msgs.msg import FormationPlanRequest, FormationVehicleSpec
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    geometry_audit_config_from_dict,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_playback import (
    VIRTUAL_MODEL_VERSION,
    VIRTUAL_PLAYBACK_BACKEND,
    VIRTUAL_PLAYBACK_SEMANTIC_VERSION,
    fields_from_timed_message,
    frame_at,
    verify_virtual_a_playback_plan,
    virtual_a_request_from_raw_request,
)

import ros_virtual_a_smoke as smoke

PLAN_ID = 92001
FORMATION_ID = "synthetic_g4"
RADIUS_M = 7.0
HALF_WIDTH_M = 3.0
RESOLUTION_M = 0.05
MARGIN_M = 3.0
VEHICLES = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
BODY_LENGTH_M = 0.738
BODY_WIDTH_M = 0.500


def arc_geometry():
    angle = -math.pi / 3.0
    start = (RADIUS_M * math.cos(angle), RADIUS_M * math.sin(angle),
             angle + math.pi / 2.0)
    angle = math.pi / 3.0
    goal = (RADIUS_M * math.cos(angle), RADIUS_M * math.sin(angle),
            angle + math.pi / 2.0)
    return start, goal


def arc_grid():
    origin = -(RADIUS_M + HALF_WIDTH_M + MARGIN_M)
    size = 2.0 * (RADIUS_M + HALF_WIDTH_M + MARGIN_M)
    count = int(math.ceil(size / RESOLUTION_M))
    coordinates = (np.arange(count) + 0.5) * RESOLUTION_M + origin
    grid_x, grid_y = np.meshgrid(coordinates, coordinates)
    radial = np.hypot(grid_x, grid_y)
    free = ((radial >= RADIUS_M - HALF_WIDTH_M)
            & (radial <= RADIUS_M + HALF_WIDTH_M)
            & (grid_x > -0.2))
    # OccupancyGrid data: 100 occupied (outside the annulus), 0 free.
    data = np.full((count, count), 100, dtype=np.int8)
    data[free] = 0
    return origin, count, data


def map_message(origin, count, data):
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.info.resolution = RESOLUTION_M
    message.info.width = count
    message.info.height = count
    message.info.origin.position.x = origin
    message.info.origin.position.y = origin
    message.info.origin.position.z = 0.0
    message.info.origin.orientation.w = 1.0
    message.data = [int(value) for value in data.reshape(-1)]
    return message


def initial_pose_message(start):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.pose.pose.position.x = float(start[0])
    message.pose.pose.position.y = float(start[1])
    quaternion = quaternion_from_euler(0.0, 0.0, float(start[2]))
    message.pose.pose.orientation.x = quaternion[0]
    message.pose.pose.orientation.y = quaternion[1]
    message.pose.pose.orientation.z = quaternion[2]
    message.pose.pose.orientation.w = quaternion[3]
    return message


def rotate(offset_x, offset_y, psi):
    cosine, sine = math.cos(psi), math.sin(psi)
    return (cosine * offset_x - sine * offset_y,
            sine * offset_x + cosine * offset_y)


def request_message(start, goal):
    message = FormationPlanRequest()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.plan_id = PLAN_ID
    message.formation_id = FORMATION_ID
    message.formation_start = Pose2D(
        x=float(start[0]), y=float(start[1]), theta=float(start[2]))
    message.formation_goal = Pose2D(
        x=float(goal[0]), y=float(goal[1]), theta=float(goal[2]))
    message.allow_reverse = False
    for vehicle_id, offset_x, offset_y in VEHICLES:
        spec = FormationVehicleSpec()
        spec.vehicle_id = vehicle_id
        spec.offset_in_formation.x = float(offset_x)
        spec.offset_in_formation.y = float(offset_y)
        spec.offset_in_formation.z = 0.0
        start_x, start_y = rotate(offset_x, offset_y, start[2])
        goal_x, goal_y = rotate(offset_x, offset_y, goal[2])
        spec.start_center = Pose2D(
            x=start[0] + start_x, y=start[1] + start_y,
            theta=float(start[2]))
        spec.goal_center = Pose2D(
            x=goal[0] + goal_x, y=goal[1] + goal_y, theta=float(goal[2]))
        spec.goal_heading_free = True
        spec.body_length_m = BODY_LENGTH_M
        spec.body_width_m = BODY_WIDTH_M
        message.vehicles.append(spec)
    return message


def marker_document(marker):
    return {
        "header": {
            "frame_id": marker.header.frame_id,
            "stamp": [int(marker.header.stamp.secs),
                      int(marker.header.stamp.nsecs)],
        },
        "ns": marker.ns,
        "id": int(marker.id),
        "type": int(marker.type),
        "action": int(marker.action),
        "pose": {
            "position": [float(marker.pose.position.x),
                         float(marker.pose.position.y),
                         float(marker.pose.position.z)],
            "orientation": [float(marker.pose.orientation.x),
                            float(marker.pose.orientation.y),
                            float(marker.pose.orientation.z),
                            float(marker.pose.orientation.w)],
        },
        "scale": [float(marker.scale.x), float(marker.scale.y),
                  float(marker.scale.z)],
        "color": [float(marker.color.r), float(marker.color.g),
                  float(marker.color.b), float(marker.color.a)],
        "lifetime": [int(marker.lifetime.secs),
                     int(marker.lifetime.nsecs)],
        "frame_locked": bool(marker.frame_locked),
        "points": [[float(point.x), float(point.y), float(point.z)]
                   for point in marker.points],
        "colors": [[float(color.r), float(color.g), float(color.b),
                    float(color.a)] for color in marker.colors],
        "text": marker.text,
    }


def marker_array_document(message):
    return {
        "frame_id": message.markers[0].header.frame_id if message.markers
        else None,
        "markers": [marker_document(marker) for marker in message.markers],
    }


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      allow_nan=False)


def marker_content(document):
    """Marker content without receipt stamps; stable for hashing/verification."""
    copied = json.loads(json.dumps(document))
    for marker in copied["markers"]:
        marker["header"].pop("stamp", None)
    return copied


def payload_hash(debug_document, corridors, guides):
    payload = {
        "planning_debug": debug_document,
        "corridors": marker_content(corridors) if corridors else None,
        "guides": marker_content(guides) if guides else None,
    }
    import hashlib
    return hashlib.sha256(canonical(payload).encode("utf-8")).hexdigest()


class Driver:
    def __init__(self, args):
        self.args = args
        self.messages = []
        self.latest = {}
        self.envelope = None
        self.envelope_document = None
        self.job_states = []
        self.verification = None
        self.raw_request = None
        self.plan = None
        self.trajectory = None
        self.capture = None
        self.started = time.monotonic()
        self.subscribers = []
        rospy.init_node("v2_corridor_scene_driver", anonymous=True)
        self.map_pub = rospy.Publisher(
            "/map", OccupancyGrid, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.request_pub = rospy.Publisher(
            "/formation/plan_request", FormationPlanRequest, queue_size=1)
        self._record_subscription(
            "/formation/planning_debug", String, self.on_planning_debug)
        self._record_subscription(
            "/formation/corridors", MarkerArray, self.on_corridors)
        self._record_subscription(
            "/formation/guides", MarkerArray, self.on_guides)
        self._record_subscription(
            "/formation/markers", MarkerArray, self.on_markers,
            record_all=False)
        self._record_subscription(
            "/formation/playback_status", String, self.on_playback_status)
        self._record_subscription(
            "/formation/playback_plan", smoke.FormationPlaybackPlan,
            self.on_envelope)
        self._record_subscription(
            "/formation/job_state", String, self.on_job_state)
        self._record_subscription(
            "/formation/optimizer_status", String, self.on_optimizer_status)

    def _record_subscription(self, topic, message_type, callback,
                             record_all=True):
        def wrapped(message):
            received = rospy.Time.now().to_sec()
            document = self.serialize(topic, message)
            self.latest[topic] = {
                "received_wall": received,
                "document": document,
            }
            if record_all:
                self.messages.append({
                    "topic": topic,
                    "received_wall": received,
                    "document": document,
                })
            callback(message, document)
        self.subscribers.append(
            (topic, rospy.Subscriber(topic, message_type, wrapped,
                                     queue_size=20)))

    def serialize(self, topic, message):
        if isinstance(message, String):
            try:
                return json.loads(message.data)
            except ValueError:
                return message.data
        if isinstance(message, MarkerArray):
            return marker_array_document(message)
        if isinstance(message, smoke.FormationPlaybackPlan):
            return smoke.envelope_document(message)
        raise TypeError("unsupported message type {}".format(type(message)))

    def on_planning_debug(self, _message, document):
        pass

    def on_corridors(self, _message, _document):
        pass

    def on_guides(self, _message, _document):
        pass

    def on_markers(self, _message, _document):
        pass

    def on_optimizer_status(self, _message, _document):
        pass

    def on_job_state(self, _message, document):
        self.job_states.append(document)

    def on_playback_status(self, _message, document):
        pass

    def on_envelope(self, message, document):
        # Admission publishes a plan_id=0 empty envelope to clear any latched
        # display; only the real plan carries the executable trajectory.
        if int(message.plan_id) != PLAN_ID or not message.trajectory.vehicles:
            return
        if self.envelope is not None:
            return
        self.envelope = message
        self.envelope_document = document

    def wait_envelope(self, timeout):
        deadline = time.monotonic() + timeout
        while self.envelope is None:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "no /formation/playback_plan within {:.1f} s".format(
                        timeout))
            rospy.sleep(0.05)

    def current_job(self):
        plan_id = int(self.envelope.plan_id)
        chosen = None
        for document in reversed(self.job_states):
            if (document.get("plan_id") == plan_id
                    and document.get("planning_context") is not None
                    and document.get("initial_context") is not None):
                chosen = document
                break
        if chosen is None:
            raise RuntimeError("no virtual job record for plan {}".format(
                plan_id))
        return chosen

    def verify_envelope(self, request_message):
        self.raw_request = smoke.raw_request_document(request_message)
        job = self.current_job()
        initial = job["initial_context"]
        context = job["planning_context"]
        grid_sha = job["grid_sha256"]
        request = virtual_a_request_from_raw_request(
            self.raw_request, initial_source=initial["source"],
            initial_context_id=initial["context_id"])
        fingerprint = request_fingerprint(request)
        if fingerprint != job.get("request_sha256"):
            raise RuntimeError("request fingerprint differs from job state")
        fields = fields_from_timed_message(self.envelope.trajectory)
        geometry_config = geometry_audit_config_from_dict(
            context.get("timing_config", {}).get("geometry_audit_config"))
        trajectory, plan, digest = verify_virtual_a_playback_plan(
            plan_id=int(self.envelope.plan_id),
            formation_id=self.envelope.formation_id,
            backend=self.envelope.backend,
            semantic_version=self.envelope.semantic_version,
            model_version=self.envelope.model_version,
            model_sha256=self.envelope.model_sha256,
            model_json=self.envelope.model_json,
            certification=self.envelope.certification,
            trajectory_fields=fields,
            expected_request=request,
            expected_context=context,
            expected_grid_sha256=grid_sha,
            geometry_audit_config=geometry_config)
        if digest != self.envelope.model_sha256:
            raise RuntimeError("verified model digest differs from message")
        self.trajectory = trajectory
        self.plan = plan
        self.verification = {
            "verified": True,
            "model_sha256": digest,
            "request_sha256": fingerprint,
            "planning_context_sha256": job.get("planning_context_sha256"),
            "grid_sha256": grid_sha,
            "duration_s": float(plan.duration_s),
            "sample_count": len(trajectory.times_from_start),
            "backend": self.envelope.backend,
            "semantic_version": self.envelope.semantic_version,
            "model_version": self.envelope.model_version,
            "certification": self.envelope.certification,
        }

    def wait_midpoint(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "playback did not reach the narrow-curve midpoint "
                    "within {:.1f} s".format(timeout))
            entry = self.latest.get("/formation/playback_status")
            if entry is not None:
                document = entry["document"]
                duration = document.get("duration_s")
                clamped = document.get("clamped_elapsed_s")
                if (document.get("state") == "PLAYING"
                        and isinstance(duration, float) and duration > 0.0
                        and isinstance(clamped, float)
                        and clamped / duration >= 0.46):
                    return entry
            rospy.sleep(0.02)

    def capture_midpoint(self, status_entry):
        debug_entry = self.latest.get("/formation/planning_debug")
        corridors_entry = self.latest.get("/formation/corridors")
        guides_entry = self.latest.get("/formation/guides")
        markers_entry = self.latest.get("/formation/markers")
        missing = [name for name, entry in (
            ("planning_debug", debug_entry),
            ("corridors", corridors_entry),
            ("guides", guides_entry),
            ("markers", markers_entry)) if entry is None]
        if missing:
            raise RuntimeError("missing display topics: {}".format(missing))
        status = status_entry["document"]
        frame = frame_at(self.trajectory, status["clamped_elapsed_s"])
        by_id = {vehicle.vehicle_id: vehicle for vehicle in frame.vehicles}
        worst_position = 0.0
        worst_theta = 0.0
        for published in status["vehicles"]:
            reference = by_id[published["vehicle_id"]]
            worst_position = max(
                worst_position,
                math.hypot(published["center"][0] - reference.center_x,
                           published["center"][1] - reference.center_y))
            error = abs(math.atan2(
                math.sin(published["theta"] - reference.theta),
                math.cos(published["theta"] - reference.theta)))
            worst_theta = max(worst_theta, error)
        if worst_position > 1e-7 or worst_theta > 1e-7:
            raise RuntimeError(
                "published playback frame differs from the authoritative "
                "model: {:.3e} m / {:.3e} rad".format(
                    worst_position, worst_theta))
        document = {
            "state": status.get("state"),
            "elapsed_s": status["elapsed_s"],
            "clamped_elapsed_s": status["clamped_elapsed_s"],
            "duration_s": status["duration_s"],
            "progress": (status["clamped_elapsed_s"]
                         / status["duration_s"]),
            "sample_fraction": status["fraction"],
            "lower_index": status["lower_index"],
            "upper_index": status["upper_index"],
            "sample_count": status["sample_count"],
            "nearest_sample_delta_s": min(
                abs(status["clamped_elapsed_s"] - value)
                for value in self.trajectory.times_from_start),
            "worst_position_error_m": worst_position,
            "worst_heading_error_rad": worst_theta,
            "model_sha256": status.get("model_sha256"),
            "plan_id": status.get("plan_id"),
            "planning_debug": debug_entry["document"],
            "corridors": corridors_entry["document"],
            "guides": guides_entry["document"],
            "markers": markers_entry["document"],
            "captured_wall": rospy.Time.now().to_sec(),
        }
        document["payload_sha256"] = payload_hash(
            debug_entry["document"],
            corridors_entry["document"],
            guides_entry["document"])
        document["markers_sha256"] = hashlib.sha256(
            canonical(marker_content(markers_entry["document"]))
            .encode("utf-8")).hexdigest()
        document["non_sample_time"] = (
            document["nearest_sample_delta_s"] > 1e-9)
        self.capture = document
        return document

    def capture_screenshot(self):
        window = str(self.args.rviz_window_id)
        subprocess.run(["xdotool", "windowraise", window], check=True)
        time.sleep(0.3)
        subprocess.run(
            ["import", "-window", window, str(self.args.screenshot)],
            check=True)
        return {
            "window_id": window,
            "path": str(self.args.screenshot),
            "bytes": os.path.getsize(self.args.screenshot),
        }

    def run(self):
        start, goal = arc_geometry()
        origin, count, data = arc_grid()
        output = {
            "schema": "virtual_a_corridor.v2.real_ros_scene.v1",
            "plan_id": PLAN_ID,
            "formation_id": FORMATION_ID,
            "scene": {
                "kind": "synthetic_annulus",
                "radius_m": RADIUS_M,
                "half_width_m": HALF_WIDTH_M,
                "resolution_m": RESOLUTION_M,
                "margin_m": MARGIN_M,
                "origin_xy": [origin, origin],
                "width": count,
                "height": count,
                "start": list(start),
                "goal": list(goal),
                "vehicles": [list(item) for item in VEHICLES],
                "body_length_m": BODY_LENGTH_M,
                "body_width_m": BODY_WIDTH_M,
            },
            "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
        }
        rospy.wait_for_message("/formation/job_state", String, timeout=15.0)
        map_publisher = self.map_pub
        map_publisher.publish(map_message(origin, count, data))
        # Wait until the planner has accepted the exact map.
        deadline = time.monotonic() + 10.0
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError("planner did not report map_ready")
            job = self.latest.get("/formation/job_state")
            if job is not None and job["document"].get("map_ready") is True:
                break
            rospy.sleep(0.1)
        self.pose_pub.publish(initial_pose_message(start))
        rospy.sleep(0.5)
        request = request_message(start, goal)
        self.request_pub.publish(request)
        output["request"] = smoke.raw_request_document(request)
        rospy.loginfo("[v2 driver] request published")
        self.wait_envelope(self.args.planning_timeout)
        rospy.loginfo("[v2 driver] envelope received; verifying")
        self.verify_envelope(request)
        rospy.loginfo("[v2 driver] envelope verified; waiting for midpoint")
        status_entry = self.wait_midpoint(self.args.playback_timeout)
        capture = self.capture_midpoint(status_entry)
        rospy.loginfo("[v2 driver] midpoint captured progress=%.6f",
                      capture["progress"])
        output["verification"] = self.verification
        output["capture"] = capture
        output["screenshot"] = self.capture_screenshot()
        output["job_state"] = self.current_job()
        return output


def write_json(path, document):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, sort_keys=True, indent=2,
                  allow_nan=False)
        handle.write("\n")


def write_messages(path, messages):
    with open(path, "w", encoding="utf-8") as handle:
        for message in messages:
            handle.write(canonical(message))
            handle.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--messages", required=True)
    parser.add_argument("--screenshot", required=True)
    parser.add_argument("--rviz-window-id", type=int, required=True)
    parser.add_argument("--planning-timeout", type=float, default=120.0)
    parser.add_argument("--playback-timeout", type=float, default=120.0)
    args = parser.parse_args()
    driver = Driver(args)
    try:
        output = driver.run()
        write_json(Path(args.output), output)
    finally:
        write_messages(Path(args.messages), driver.messages)
    print(canonical({
        "ok": True,
        "payload_sha256": output["capture"]["payload_sha256"],
        "progress": output["capture"]["progress"],
        "elapsed_s": output["capture"]["clamped_elapsed_s"],
    }))


if __name__ == "__main__":
    main()
