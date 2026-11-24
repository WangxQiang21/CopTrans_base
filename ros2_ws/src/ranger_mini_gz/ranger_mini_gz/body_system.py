"""Harmonic system bridging independent Twist input to a static model's body."""
from pathlib import Path
from threading import Lock
import time

from gz.msgs10.int32_pb2 import Int32
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.sim8 import Model
from gz.transport13 import Node
import yaml

from ranger_mini_gz.body_sim import GazeboBody, PlantSettings, ZERO


class BodyTwistSystem:
    def configure(self, entity, sdf, ecm, _event_manager):
        config = yaml.safe_load(Path(sdf.get_string("settings")).read_text())
        settings = PlantSettings(**config["plant"])
        self.body = GazeboBody(entity, ecm, settings, sdf.get_bool("managed_switching"))
        self.name = Model(entity).name(ecm)
        self.timeout = settings.command_timeout
        self.lock = Lock()
        self.command = ZERO
        self.request = None
        self.sim_time = 0.
        self.command_time = self.request_time = self.command_wall = -float("inf")
        self.transport = Node()
        self.transport.subscribe(Twist, f"/{self.name}/cmd_vel", self.on_twist)
        self.transport.subscribe(Int32, f"/{self.name}/sim_request_mode", self.on_mode_request)
        self.odom = self.transport.advertise(f"/{self.name}/odom", Odometry)
        self.mode = self.transport.advertise(f"/{self.name}/motion_state", Int32)

    def on_twist(self, message):
        with self.lock:
            self.command = (message.linear.x, message.linear.y, message.angular.z)
            self.command_time = self.sim_time
            self.command_wall = time.monotonic()

    def on_mode_request(self, message):
        with self.lock:
            self.request = message.data
            self.request_time = self.sim_time

    def pre_update(self, info, ecm):
        if info.paused or not hasattr(self, "body"):
            return
        now = info.sim_time.total_seconds()
        with self.lock:
            self.sim_time = now
            fresh = (0. <= now - self.command_time <= self.timeout
                     and time.monotonic() - self.command_wall <= self.timeout)
            command = self.command if fresh else ZERO
            request = self.request if fresh and 0. <= now - self.request_time <= self.timeout else None
        self.body.pre_update(now, info.dt.total_seconds(), ecm, command, request)

    def post_update(self, info, ecm):
        if info.paused or not hasattr(self, "body"):
            return
        state = self.body.post_update(info.sim_time.total_seconds(), ecm)
        if state is None:
            return
        msg = Odometry()
        stamp = round(state.stamp * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nsec = divmod(stamp, 10**9)
        frame = msg.header.data.add()
        frame.key = "frame_id"
        frame.value.append("map")
        child = msg.header.data.add()
        child.key = "child_frame_id"
        child.value.append(f"{self.name}/base_link")
        p, q = state.pose.pos(), state.pose.rot()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x(), p.y(), p.z()
        msg.pose.orientation.x, msg.pose.orientation.y = q.x(), q.y()
        msg.pose.orientation.z, msg.pose.orientation.w = q.z(), q.w()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = state.velocity
        self.odom.publish(msg)
        self.mode.publish(Int32(data=state.mode))


def get_system():
    return BodyTwistSystem()
