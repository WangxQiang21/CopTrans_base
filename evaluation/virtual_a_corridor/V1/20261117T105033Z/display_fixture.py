#!/usr/bin/env python3
"""V1 display-only fixture: publish one audited C1 chain to playback.

This is test tooling, not a production planner node and not a plan source.
It supplies a real ROS request and a matching job record so the playback
admission check is exercised; the chain itself is the saved C1 audit output,
copied verbatim by corridor_display_fields().  No optimizer result is faked.
"""
import argparse
import json

import rospy
from geometry_msgs.msg import Point, Pose2D
from std_msgs.msg import Header, String
from fwis_v3_msgs.msg import FormationPlanRequest, FormationVehicleSpec

from fwis_v3_optimizer.formation_continuous_a.contracts import (
    ConvexCorridor,
    CorridorChain,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    CORRIDOR_DISPLAY_STATE,
    corridor_display_fields,
)


def load_chain(path, region_limit):
    document = json.loads(open(path, encoding="utf-8").read())
    payload = document["chain"]
    regions = tuple(ConvexCorridor(
        seed_start=tuple(region["seed_start"]),
        seed_end=tuple(region["seed_end"]),
        A=tuple(tuple(row) for row in region["A"]),
        b=tuple(region["b"]),
        vertices=tuple(tuple(vertex) for vertex in region["vertices"]))
        for region in payload["regions"])
    if region_limit:
        regions = regions[:region_limit]
    chain = CorridorChain(
        regions=regions, route_source=payload["route_source"],
        grid_sha256=payload["grid_sha256"])
    return document, chain


def request_message(plan_id, formation_id):
    message = FormationPlanRequest()
    message.header = Header()
    message.header.frame_id = "map"
    message.header.stamp = rospy.Time.now()
    message.plan_id = plan_id
    message.formation_id = formation_id
    message.formation_start = Pose2D(0.0, 0.0, 0.0)
    message.formation_goal = Pose2D(3.0, 0.0, 0.0)
    message.allow_reverse = False
    for vehicle_id, (offset_x, offset_y) in enumerate(
            ((1.0, 0.0), (0.0, 1.0), (0.0, -1.0)), start=1):
        spec = FormationVehicleSpec()
        spec.vehicle_id = vehicle_id
        spec.offset_in_formation = Point(offset_x, offset_y, 0.0)
        spec.start_center = Pose2D(offset_x, offset_y, 0.0)
        spec.goal_center = Pose2D(3.0 + offset_x, offset_y, 0.0)
        spec.goal_heading_free = True
        spec.body_length_m = 0.738
        spec.body_width_m = 0.5
        message.vehicles.append(spec)
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--chain-json", required=True)
    parser.add_argument("--plan-id", type=int, required=True)
    parser.add_argument("--generation", type=int, required=True)
    parser.add_argument("--route-id", type=int, default=0)
    parser.add_argument("--formation-id", default="wedge_3")
    parser.add_argument("--context-sha", default="c" * 64)
    parser.add_argument("--region-limit", type=int, default=0)
    parser.add_argument("--skip-request", action="store_true")
    parser.add_argument("--skip-job", action="store_true")
    parser.add_argument("--clear", action="store_true")
    args = parser.parse_args()
    c1, chain = load_chain(args.chain_json, args.region_limit)
    if len(args.context_sha) != 64:
        raise SystemExit("--context-sha must be 64 characters")
    identity = {
        "plan_id": args.plan_id,
        "formation_id": args.formation_id,
        "backend": "virtual_rigid_a",
        "generation": args.generation,
        "route_id": args.route_id,
        "grid_sha256": chain.grid_sha256,
        "planning_context_sha256": args.context_sha,
        "model_sha256": None,
    }
    display = corridor_display_fields(chain, identity)
    document = {
        "state": CORRIDOR_DISPLAY_STATE,
        "plan_id": args.plan_id,
        "formation_id": args.formation_id,
        "backend": "virtual_rigid_a",
        "generation": args.generation,
        "grid_sha256": chain.grid_sha256,
        "planning_context_sha256": args.context_sha,
        "model_sha256": None,
        "corridors": [display],
    }
    rospy.init_node("v1_corridor_display_fixture", anonymous=True)
    request_pub = rospy.Publisher(
        "/formation/plan_request", FormationPlanRequest, queue_size=1)
    job_pub = rospy.Publisher(
        "/formation/job_state", String, queue_size=1)
    debug_pub = rospy.Publisher(
        "/formation/planning_debug", String, queue_size=1, latch=True)
    rospy.sleep(0.5)
    request = request_message(args.plan_id, args.formation_id)
    if not args.skip_request:
        request_pub.publish(request)
        rospy.sleep(0.3)
    if not args.skip_job:
        job = {
            "schema": "formation_job/v1",
            "state": "PLANNING",
            "backend": "virtual_rigid_a",
            "generation": args.generation,
            "plan_id": args.plan_id,
            "formation_id": args.formation_id,
            "model_sha256": None,
            "request_sha256": None,
            "planning_context_sha256": args.context_sha,
            "grid_sha256": chain.grid_sha256,
            "reason": "",
            "detail": "",
        }
        job_pub.publish(String(data=json.dumps(job, sort_keys=True)))
        rospy.sleep(0.3)
    if args.clear:
        cleared = {
            "state": "CLEARED",
            "plan_id": args.plan_id,
            "formation_id": args.formation_id,
            "backend": "virtual_rigid_a",
            "generation": args.generation,
            "grid_sha256": None,
            "planning_context_sha256": None,
            "model_sha256": None,
            "corridors": [],
        }
        debug_pub.publish(String(data=json.dumps(cleared, sort_keys=True)))
        action = "cleared"
    elif args.skip_job and args.skip_request:
        debug_pub.publish(String(data=json.dumps(document, sort_keys=True)))
        action = "stale-debug"
    else:
        debug_pub.publish(String(data=json.dumps(document, sort_keys=True)))
        action = "displayed"
    rospy.sleep(0.5)
    print(json.dumps({
        "action": action,
        "plan_id": args.plan_id,
        "generation": args.generation,
        "route_id": args.route_id,
        "formation_id": args.formation_id,
        "chain_digest": chain.digest,
        "grid_sha256": chain.grid_sha256,
        "planning_context_sha256": args.context_sha,
        "c1_sample_grid_sha256": c1["chain"]["grid_sha256"],
        "c1_sample_digest": c1["chain"]["digest"],
        "regions": len(chain.regions),
        "state": (None if args.clear else CORRIDOR_DISPLAY_STATE),
        "corridors": ([] if args.clear else [display]),
    }, sort_keys=True))


if __name__ == "__main__":
    main()