"""Decide when both native request receivers share a map and advancing ROS clock."""
import json


class NativeReadiness:
    def __init__(self):
        self.reports = {}

    def observe(self, role, text, wall):
        try:
            msg=json.loads(text)
            if (msg["ready"] is not True or type(msg["clock_ns"]) is not int or
                    not isinstance(msg["map_sha256"],str) or len(msg["map_sha256"])!=64):
                self.reports.pop(role,None)
                return
            self.reports[role]=(wall,msg["clock_ns"],msg["map_sha256"],msg.get("solver_build"))
        except (KeyError,TypeError,ValueError):
            self.reports.pop(role,None)

    def ready(self, ros_ns, wall):
        if ros_ns<=0 or self.reports.keys()!={"planner","executor"}:
            return False
        return (len({r[2] for r in self.reports.values()})==1 and
                all(0<=wall-received<=.20 and stamp>0 and abs(ros_ns-stamp)<=200000000
                    for received,stamp,_,_build in self.reports.values()))

    def identities(self):
        identities = {(value[2], value[3]) for value in self.reports.values()}
        if self.reports.keys() != {"planner", "executor"} or len(identities) != 1:
            raise ValueError("Native planner and executor do not agree on map and build identity")
        map_hash, build = identities.pop()
        if not isinstance(build, str) or len(build) != 64:
            raise ValueError("Native solver build digest unavailable")
        return map_hash, build
