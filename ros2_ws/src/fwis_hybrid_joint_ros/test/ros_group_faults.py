#!/usr/bin/env python3
"""Exercise the installed native ROS chain with a synthetic feedback plant and injected faults."""
import argparse
from copy import deepcopy
import json
from math import cos, sin, isfinite
import os
import signal
from pathlib import Path
import subprocess
import sys
from time import monotonic
from tempfile import TemporaryDirectory
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from fwis_v3_msgs.msg import FormationPlanRequest, FormationPlaybackPlan, FormationVehicleSpec, HybridExecutionPermit
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String
from rosgraph_msgs.msg import Clock
from rclpy.time import Time

DURABLE = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
OFFSETS = ((1., 0.), (0., 1.), (0., -1.))


class Run(Node):
    def __init__(self, count, profile):
        super().__init__("hybrid_group_fault_fixture")
        self.count = count
        self.profile = profile
        self.temporary = TemporaryDirectory(prefix="coptrans-ros-faults-")
        registry_path = Path(get_package_share_directory("fwis_bringup")) / "config" / "test_defaults.yaml"
        registry = yaml.safe_load(registry_path.read_text())
        speed = registry["speed_profiles"][profile]
        task_name = "multi" if count == 3 else "single"
        self.overrides = speed.get(task_name,{})
        control_path = Path(get_package_share_directory("fwis_control")) / "config" / "ranger_tracking.yaml"
        control = yaml.safe_load(control_path.read_text())
        control["limits"].update(speed.get("controller",{}))
        self.tracker_config = Path(self.temporary.name) / "tracker.yaml"
        self.tracker_config.write_text(yaml.safe_dump(control))
        self.cancel = self.create_publisher(String,"/hybrid_joint/cancel",1)
        self.children, self.events, self.permits, self.feedback = [], [], [], set()
        self.disabled = set()
        self.commands, self.command_events, self.modes, self.last_plan = {}, [], {}, None
        self.model_topic = "/formation/playback_plan" if count == 3 else "/hybrid_joint/plan"
        self.poses = [list(offset) + [0.] for offset in (OFFSETS if count == 3 else ((0., 0.),))]
        self.expired_fixture = False
        self.clock_fixture = False
        self.repeat_ack = False
        self.ack_before_envelope = False
        self.group_plan = self.create_publisher(FormationPlaybackPlan, "/fixture/group_plan", DURABLE)
        self.status_relays = []
        self.first_admission = set()
        self.clock_origin = monotonic()
        self.rollback = 0.
        self.bad_numeric = set()
        self.wrong_permit = None
        self.nonfinite_commands = 0
        self.check_reasons = {}
        self.fake_clock = self.create_publisher(Clock, "/clock", 5)
        self.bad_permit = self.create_publisher(HybridExecutionPermit,"/hybrid_joint/execution_permit",1)
        self.original = None
        self.last_good_feedback = {}
        self.unsafe_after_stale = 0
        self.goal = self.create_publisher(FormationPlanRequest if count == 3 else PoseStamped,
                                          "/formation/plan_request" if count == 3 else "/move_base_simple/goal", 10)
        self.start = self.create_publisher(PoseStamped, "/start_pose", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", DURABLE)
        self.inject = self.create_publisher(FormationPlaybackPlan, self.model_topic, DURABLE)
        self.create_subscription(FormationPlaybackPlan, self.model_topic, self.model, DURABLE)
        self.create_subscription(String, "/hybrid_joint/admission", self.admission, DURABLE)
        self.create_subscription(HybridExecutionPermit, "/hybrid_joint/execution_permit",
                                 lambda m: self.permits.append((monotonic(), m.plan_id, m.enabled)), 10)
        self.outputs = []
        for i in range(count):
            self.status_relays.append(self.create_publisher(String,f"/fixture/ack{i+1}",30))
            prefix = f"/robot{i+1}"
            self.outputs.append((self.create_publisher(Odometry, prefix + "/odom", 5),
                                 self.create_publisher(Int32, prefix + "/motion_state", 5)))
            self.commands[i] = (0., 0., 0.)
            self.modes[i] = 0
            self.create_subscription(Twist, prefix + "/cmd_vel", lambda m, i=i: self.cmd(i, m), 10)
            self.create_subscription(Int32, prefix + "/sim_request_mode", lambda m, i=i: self.mode(i, m), 5)
            self.create_subscription(String, prefix + "/controller_status", lambda m, i=i: self.status(i, m), 30)
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.width = self.map.info.height = 80
        self.map.info.resolution = .25
        self.map.info.origin.position.x = self.map.info.origin.position.y = -10.
        self.map.info.origin.orientation.w = 1.
        self.map.data = [0] * (80 * 80)
        self.map_pub.publish(self.map)
        self.previous = monotonic()
        self.create_timer(.02, self.update)

    def spawn(self):
        def launch(package, name, args):
            path = Path(get_package_prefix(package)) / "lib" / package / name
            self.children.append(subprocess.Popen([str(path), "--ros-args", *args]))
        parameters=["-p",f"speed_profile:={self.profile}"]
        clock_args = ["-p","use_sim_time:=true"] if self.clock_fixture else []
        for key,value in self.overrides.items():
            prefix="timing_limits." if self.count==3 else "quality.limits."
            if key!="target_v":
                parameters.extend(["-p",f"{prefix}{key}:={value}"])
        if self.expired_fixture:
            self.create_subscription(FormationPlaybackPlan, "/fixture/delayed_plan", self.generated, DURABLE)
        for role in ("planner", "executor"):
            remaps = (["-r",f"{self.model_topic}:=/fixture/delayed_plan"]
                      if self.expired_fixture and role=="planner" else [])
            if self.repeat_ack and role=="executor":
                for i in range(self.count):
                    remaps.extend(["-r",f"/robot{i+1}/controller_status:=/fixture/ack{i+1}"])
            if self.ack_before_envelope and role=="executor":
                remaps.extend(["-r",f"{self.model_topic}:=/fixture/group_plan"])
            launch("fwis_hybrid_joint_ros", f"hybrid_joint_{role}", [
                "-p", "task:=" + ("formation" if self.count == 3 else "single"), *parameters, *remaps, *clock_args])
        for i in range(self.count):
            launch("fwis_control", "ranger_tracking_controller_cpp", [
                "-r", f"__ns:=/robot{i + 1}", "-p", "hybrid_joint:=true", "-p", f"vehicle_id:={i+1}",
                "-p", f"hybrid_plan_topic:={self.model_topic}", "-p", "permit_topic:=/hybrid_joint/execution_permit",
                "-p", "future_start_window_s:=1.35", "-p", f"config_file:={self.tracker_config}",*clock_args])

    def generated(self, msg):
        self.original = msg

    def model(self, m):
        if self.last_plan is None or self.last_plan.plan_id != m.plan_id:
            self.plan_received_wall = monotonic()
        self.last_plan = m

    def admission(self, m):
        self.events.append((monotonic(), json.loads(m.data)))

    def status(self, i, m):
        row = json.loads(m.data)
        if row.get("kind")=="admission" and row.get("result") in ("PASS","WARN"):
            self.first_admission.add((row["plan_id"],i))
        if (self.repeat_ack and row.get("kind")=="execution"
                and any(e["phase"]=="PRECHECK" and e["plan_id"]==row["plan_id"] for _,e in self.events)):
            self.status_relays[i].publish(m)
        if row.get("phase") in ("STOPPED","REJECTED") and row.get("reasons"):
            self.check_reasons.setdefault(row["plan_id"],set()).update(row["reasons"])
            print(f"tracker {i+1} stopped {row['reasons']}",flush=True)
        if (i in self.disabled and i in self.last_good_feedback and row.get("kind")=="execution"
                and row["stamp"] - self.last_good_feedback[i] > .200001
                and any(abs(v)>1e-9 for v in row["command"])):
            self.unsafe_after_stale += 1
        if row.get("phase") == "COMPLETE":
            self.feedback.add((row["plan_id"], i))

    def cmd(self, i, m):
        self.commands[i] = m.linear.x, m.linear.y, m.angular.z
        if not all(isfinite(v) for v in self.commands[i]):
            self.nonfinite_commands += 1
        self.command_events.append((monotonic(), i, max(map(abs, self.commands[i]))))

    def mode(self, i, m):
        if m.data >= 0:
            self.modes[i] = m.data

    def update(self):
        wall = monotonic()
        dt = min(wall - self.previous, .10)
        self.previous = wall
        time = (Time(nanoseconds=round((wall-self.clock_origin + 1.-self.rollback)*1e9))
                if self.clock_fixture else self.get_clock().now())
        stamp = time.to_msg()
        if self.clock_fixture:
            self.fake_clock.publish(Clock(clock=stamp))
        if self.wrong_permit and self.last_plan:
            bad = HybridExecutionPermit(plan_id=self.last_plan.plan_id, start=self.last_plan.header.stamp,
                                        model_sha256=self.last_plan.model_sha256, stamp=stamp, enabled=True)
            if self.wrong_permit=="wrong_epoch":
                bad.start=Time(nanoseconds=self.last_plan.header.stamp.sec*10**9+self.last_plan.header.stamp.nanosec+10**9).to_msg()
            elif self.wrong_permit=="future_permit":
                bad.stamp=Time(nanoseconds=time.nanoseconds+10**9).to_msg()
            else:
                bad.enabled=False
            self.bad_permit.publish(bad)
        for i, (odom, modes) in enumerate(self.outputs):
            p = self.poses[i]
            vx, vy, wz = self.commands[i]
            p[0] += (cos(p[2])*vx-sin(p[2])*vy)*dt
            p[1] += (sin(p[2])*vx+cos(p[2])*vy)*dt
            p[2] += wz*dt
            if i not in self.disabled:
                m = Odometry()
                m.header.frame_id, m.child_frame_id = "map", f"robot{i+1}/base_link"
                m.header.stamp = stamp
                m.pose.pose.position.x, m.pose.pose.position.y = p[:2]
                m.pose.pose.orientation.w, m.pose.pose.orientation.z = cos(p[2]/2), sin(p[2]/2)
                m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = vx, vy, wz
                if i in self.bad_numeric:
                    m.twist.twist.linear.x = float("nan")
                self.last_good_feedback[i] = stamp.sec + stamp.nanosec*1e-9
                odom.publish(m)
                modes.publish(Int32(data=self.modes[i]))

    def until(self, predicate, timeout):
        end = monotonic() + timeout
        while monotonic() < end:
            if any(p.poll() is not None for p in self.children):
                raise RuntimeError("native process exited")
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=.02)
        raise TimeoutError(str(self.events[-5:]))

    def request(self, plan_id):
        self.map_pub.publish(self.map)
        if self.count == 1:
            start = PoseStamped()
            start.header.frame_id = "map"
            start.pose.orientation.w = 1.
            start.pose.position.x, start.pose.position.y = self.poses[0][:2]
            self.start.publish(start)
            self.pump(.12)
            goal = deepcopy(start)
            goal.pose.position.x += .24
            self.goal.publish(goal)
        else:
            m = FormationPlanRequest(plan_id=plan_id, formation_id="wedge", allow_reverse=True)
            m.header.frame_id = "map"
            x, y = self.poses[0][0] - OFFSETS[0][0], self.poses[0][1]
            m.formation_start = Pose2D(x=x, y=y)
            m.formation_goal = Pose2D(x=x+.24, y=y)
            for i, (dx, dy) in enumerate(OFFSETS):
                m.vehicles.append(FormationVehicleSpec(vehicle_id=i+1, goal_heading_free=True,
                    offset_in_formation=Point(x=dx, y=dy), start_center=Pose2D(x=x+dx, y=y+dy),
                    goal_center=Pose2D(x=x+dx+.24, y=y+dy), body_length_m=.738, body_width_m=.5))
            self.goal.publish(m)

    def pump(self, duration):
        until = monotonic() + duration
        while monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.02)

    def exercise(self, name, plan_id):
        if name == "missing_member":
            self.disabled = {self.count - 1}
            self.pump(.28)
        self.unsafe_after_stale = 0
        self.nonfinite_commands = 0
        self.request(plan_id)
        if name=="expired":
            self.until(lambda:self.original and self.original.plan_id==plan_id,12.)
            self.until(lambda:self.get_clock().now().nanoseconds >
                       self.original.header.stamp.sec*10**9+self.original.header.stamp.nanosec+10**8,3.)
            self.inject.publish(self.original)
        self.until(lambda: self.last_plan and self.last_plan.plan_id == plan_id, 12)
        if name=="ack_before_envelope":
            self.until(lambda:all((plan_id,i) in self.first_admission for i in range(self.count)),1.)
            self.group_plan.publish(self.last_plan)
        if name == "admission_only":
            self.until(lambda:any(e["plan_id"]==plan_id and e["phase"]=="AUTHORIZED" for _,e in self.events),2.)
            event=next(e for _,e in self.events if e["plan_id"]==plan_id and e["phase"]=="AUTHORIZED")
            if not 0. < event["delivery_and_admission_wall_s"] < 1.25:
                raise AssertionError("invalid measured delivery interval")
            self.cancel.publish(String(data="admission_benchmark_reset"))
            self.until(lambda:any(e["plan_id"]==plan_id and e["phase"]=="STOPPED" for _,e in self.events),2.)
            self.pump(.15)
            return {"task":self.count,"profile":self.profile,"fault":name,"id":plan_id,
                    "delivery_and_admission_wall_s":event["delivery_and_admission_wall_s"]}
        if name == "tamper":
            bad = deepcopy(self.last_plan)
            bad.model_json += " "
            self.inject.publish(bad)
        if name in ("stale_feedback","wrong_mode","map_replaced","permit_loss","clock_rollback","nan_feedback","wrong_epoch","future_permit","revoked_permit"):
            self.until(lambda: all(abs(c[0]) > .01 for c in self.commands.values()), 4.)
        if name=="clock_rollback":
            self.rollback+=.8
        if name=="nan_feedback":
            self.bad_numeric={self.count-1}
        if name=="wrong_mode":
            self.modes[self.count-1]=2
        if name=="map_replaced":
            self.map.data[0]=100 if not self.map.data[0] else 0
            self.map_pub.publish(self.map)
        if name in ("permit_loss","wrong_epoch","future_permit","revoked_permit"):
            self.wrong_permit = name if name!="permit_loss" else None
            coordinator=self.children[1]
            coordinator.send_signal(signal.SIGSTOP)
            try:
                self.pump(.38)
                if any(abs(v)>1e-9 for c in self.commands.values() for v in c):
                    raise AssertionError("motion continued without group permit")
            finally:
                self.wrong_permit=None
                coordinator.send_signal(signal.SIGCONT)
        if name == "stale_feedback":
            self.disabled = {self.count - 1}
        expected = "COMPLETE" if name in ("success","repeat_ack","ack_before_envelope") else "STOPPED"
        self.until(lambda: any(e["plan_id"] == plan_id and e["phase"] == expected for _, e in self.events), 12.)
        self.pump(.3)
        final = next(e for _, e in self.events if e["plan_id"] == plan_id and e["phase"] == expected)
        release = [t for t,e in self.events if e["plan_id"] == plan_id and e["phase"] == "AUTHORIZED"]
        if name == "missing_member" and any(allow for _, i, allow in self.permits if i==plan_id):
            raise AssertionError("partial group received motion permission")
        if any(abs(v) > 1e-9 for command in self.commands.values() for v in command):
            raise AssertionError("nonzero group output after stop")
        if expected == "COMPLETE" and not all((plan_id,i) in self.feedback for i in range(self.count)):
            raise AssertionError("incomplete group")
        if self.unsafe_after_stale:
            raise AssertionError("motion published with stale feedback")
        if self.nonfinite_commands:
            raise AssertionError("nonfinite command reached the plant")
        self.disabled.clear()
        self.bad_numeric.clear()
        self.modes=dict.fromkeys(self.modes,0)
        return {"task": self.count, "profile":self.profile, "fault": name, "id": plan_id, "phase": expected, "reason": final["reason"],
                "release_s": release[0]-self.plan_received_wall if release else None,
                "stale_feedback_moving_commands":self.unsafe_after_stale,
                "nonfinite_commands":self.nonfinite_commands,
                "tracker_reasons":sorted(self.check_reasons.get(plan_id,()))}

    def close(self):
        for p in self.children:
            p.terminate()
        for p in self.children:
            try: p.wait(4)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--vehicles", type=int, choices=(1,3), required=True)
    parser.add_argument("--speed-profile",choices=("baseline","standard"),default="baseline")
    parser.add_argument("--admission-trials",type=int,default=0)
    parser.add_argument("--cases", nargs="+", default=["success","tamper","missing_member","stale_feedback"])
    parser.add_argument("--out", type=Path, required=True)
    args=parser.parse_args()
    if not os.environ.get("ROS_DOMAIN_ID"):
        sys.exit("Set an isolated ROS_DOMAIN_ID for synthetic robot topics")
    rclpy.init()
    node=Run(args.vehicles,args.speed_profile)
    if args.cases==["repeat_ack"]:
        node.repeat_ack=True
    if args.cases==["ack_before_envelope"]:
        node.ack_before_envelope=True
    if "clock_rollback" in args.cases:
        if args.cases!=["clock_rollback"]:
            parser.error("clock rollback runs on a dedicated simulated clock")
        node.clock_fixture=True
    if "expired" in args.cases:
        if args.cases!=["expired"]:
            parser.error("the expired-delivery fixture needs a dedicated relay process")
        node.expired_fixture=True
    rows=[]
    try:
        node.spawn()
        node.until(lambda: node.goal.get_subscription_count()>=2 and
                   node.map_pub.get_subscription_count()>=args.vehicles+2 and
                   len(node.command_events)>args.vehicles*3, 12.)
        cases=(["admission_only"]*(args.admission_trials+5) if args.admission_trials else args.cases)
        for i, case in enumerate(cases,1):
            row=node.exercise(case,i)
            if args.admission_trials: row["warmup"]=i<=5
            rows.append(row)
            print(json.dumps(rows[-1]),flush=True)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(rows,indent=2)+"\n")
if __name__=="__main__":
    main()
