"""Planning status semantics shared with the playback HUD, without ROS types."""
from copy import deepcopy


class PlanningDisplayState:
    _rank = {"pending": 0, "TRAJECTORY_RECEIVED": 0, "REJECTED": 1,
             "GEOMETRY_ONLY": 2, "FEASIBLE": 3, "LEGACY_NATIVE_PUBLISHED": 3}

    def __init__(self):
        self.report = {"plan_id": 0, "status": "IDLE"}

    def accept_report(self, report):
        if not isinstance(report, dict):
            return False
        plan_id, status = report.get("plan_id"), report.get("status")
        if (type(plan_id) is not int or plan_id < self.report["plan_id"] or
                not isinstance(status, str) or status not in self._rank):
            return False
        if (plan_id == self.report["plan_id"] and self._rank[status] <
                self._rank.get(self.report["status"], -1)):
            return False
        self.report = deepcopy({k: report[k] for k in
            ("plan_id", "status", "start", "reason") if k in report})
        return True

    def accept_trajectory(self, plan_id):
        if type(plan_id) is not int or plan_id < self.report["plan_id"]:
            return False
        if (plan_id == self.report["plan_id"] and
                self.report["status"] in ("GEOMETRY_ONLY", "REJECTED")):
            return False
        if plan_id > self.report["plan_id"]:
            self.report = {"plan_id": plan_id, "status": "TRAJECTORY_RECEIVED"}
        return True

    def text(self):
        return {"pending": "PLANNING",
                "GEOMETRY_ONLY": "GEOMETRY ONLY / EXECUTION NOT CERTIFIED",
                "REJECTED": "NO CERTIFIED PATH",
                "LEGACY_NATIVE_PUBLISHED": "LEGACY NATIVE / RESTRICTED PROFILE UNVERIFIED",
                "TRAJECTORY_RECEIVED": "AWAITING PLAN REPORT"}.get(self.report["status"], "")
