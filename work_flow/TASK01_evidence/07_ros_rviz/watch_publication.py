#!/usr/bin/env python3
"""Record every publication attempt seen on the Step 6 review topics.

The recorder is diagnostic only: it subscribes to the planner's executable
topic (``/formation/playback_plan``) and the debug topic
(``/formation/planning_debug``) and appends one JSON line per received
message.  Each line distinguishes an executable atomic envelope from the
planned invalid placeholder used to clear a stale latched animation when a
plan has no executable envelope.

usage: watch_publication.py <jsonl-output>
"""
import json
import sys
import time

import rospy
from fwis_v3_msgs.msg import FormationPlaybackPlan
from std_msgs.msg import String

PLAN_TOPIC = "/formation/playback_plan"
DEBUG_TOPIC = "/formation/planning_debug"
STATUS_TOPIC = "/formation/optimizer_status"


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: watch_publication.py <jsonl-output>")
    output = open(sys.argv[1], "a", buffering=1)
    start = time.monotonic()

    def emit(record):
        record["recv_monotonic_s"] = time.monotonic() - start
        record["recv_wall_s"] = time.time()
        output.write(json.dumps(record, sort_keys=True) + "\n")

    def on_plan(message):
        emit({
            "topic": PLAN_TOPIC,
            "plan_id": int(message.plan_id),
            "formation_id": message.formation_id,
            "backend": message.backend,
            "semantic_version": message.semantic_version,
            "model_version": message.model_version,
            "model_sha256": message.model_sha256,
            "model_json_length": len(message.model_json or ""),
            "sample_count": len(message.trajectory.times_from_start),
            "executable": bool(message.model_json) and
            len(message.trajectory.times_from_start) > 0,
        })

    def on_debug(message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            document = {}
        emit({
            "topic": DEBUG_TOPIC,
            "plan_id": document.get("plan_id"),
            "state": document.get("state"),
            "stage": document.get("stage"),
            "failure": document.get("failure"),
            "candidate_count": (len(document.get("candidates"))
                                if isinstance(document.get("candidates"),
                                              list) else None),
            "corridor_round": document.get("corridor_round"),
            "round": document.get("round"),
            "document_keys": sorted(document.keys()),
        })

    def on_status(message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            document = {}
        emit({
            "topic": STATUS_TOPIC,
            "plan_id": document.get("plan_id"),
            "event": document.get("event"),
            "state": document.get("job_state"),
            "status": document.get("status"),
            "reason": document.get("reason"),
            "feasible": document.get("feasible"),
            "executable": document.get("executable"),
            "published": document.get("trajectory_published"),
            "pipeline_status": document.get("pipeline_status"),
            "model_sha256": document.get("model_sha256"),
            "virtual_result": document.get("virtual_result"),
        })

    rospy.init_node("task01_publication_watch", anonymous=True)
    rospy.Subscriber(PLAN_TOPIC, FormationPlaybackPlan, on_plan, queue_size=100)
    rospy.Subscriber(DEBUG_TOPIC, String, on_debug, queue_size=100)
    rospy.Subscriber(STATUS_TOPIC, String, on_status, queue_size=100)
    rospy.loginfo("[task01 watch] recording %s and %s to %s",
                  PLAN_TOPIC, DEBUG_TOPIC, sys.argv[1])
    rospy.spin()
    output.close()


if __name__ == "__main__":
    main()