#!/usr/bin/env python3
"""S3 validation-only ROS adapter for multi-vehicle formation requests.

The node subscribes to ``/formation/plan_request`` and publishes a structured
JSON status on ``/formation/status``.  It deliberately starts no search, emits
no ``FormationPlanSeed`` or ``FormationTimedTrajectory``, and never claims
``FEASIBLE`` or executability.  Because there is no successful output stream
yet, a new request cannot leave a stale successful trajectory published.
"""
import json

import rospy
from std_msgs.msg import String

from fwis_v3_msgs.msg import FormationPlanRequest
from fwis_v3_optimizer.formation_request import (INVALID_REQUEST,
                                                 RequestValidation,
                                                 validate_formation_request)


class FormationOptimizerNode:
    """Thin ROS boundary around the ROS-independent request validator."""

    REQUEST_TOPIC = "/formation/plan_request"
    STATUS_TOPIC = "/formation/status"

    def __init__(self):
        self.requests_seen = 0
        self.last_plan_id = None
        self.status_pub = rospy.Publisher(
            self.STATUS_TOPIC, String, queue_size=1, latch=True)
        rospy.Subscriber(self.REQUEST_TOPIC, FormationPlanRequest,
                         self.on_request, queue_size=1)

    @staticmethod
    def _identity(message):
        plan_id = getattr(message, "plan_id", None)
        formation_id = getattr(message, "formation_id", None)
        return (plan_id if isinstance(plan_id, int) and
                not isinstance(plan_id, bool) else None,
                formation_id if isinstance(formation_id, str) else None)

    def on_request(self, message):
        self.requests_seen += 1
        plan_id, formation_id = self._identity(message)
        try:
            result = validate_formation_request(message)
        except Exception as exc:  # the validator itself is fail-closed
            result = RequestValidation(
                False, INVALID_REQUEST, "request", None,
                "node-level validation failure: {!r}".format(exc))
        status = result.as_status()
        status.update({
            "plan_id": plan_id,
            "formation_id": formation_id,
            "node_state": ("VALIDATED_NOT_PLANNED" if result.valid
                           else "REJECTED"),
            "requests_seen": self.requests_seen,
        })
        if result.valid:
            self.last_plan_id = result.snapshot.plan_id
            rospy.loginfo(
                "[formation validator] plan_id=%s formation_id=%s vehicles=%s "
                "validated; search_started=false trajectory_published=false",
                plan_id, formation_id, result.snapshot.vehicle_ids)
        else:
            rospy.logwarn(
                "[formation validator] plan_id=%s formation_id=%s rejected "
                "reason=%s field=%s vehicle_id=%s detail=%s",
                plan_id, formation_id, result.reason, result.field,
                result.vehicle_id, result.detail)
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))


if __name__ == "__main__":
    rospy.init_node("fwis_v3_formation_optimizer")
    FormationOptimizerNode()
    rospy.spin()
