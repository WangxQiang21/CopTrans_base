"""Gazebo body-level plant for two Ranger Mini chassis carrying a rigid blade."""

import math
from pathlib import Path
from threading import Lock
from time import monotonic

from gz.math7 import Pose3d
from gz.msgs10.int32_pb2 import Int32
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.sim8 import Link, Model, World
from gz.transport13 import Node
import yaml

from ranger_mini_gz.body_sim import ACKERMANN, OBLIQUE, SPIN, PlantSettings, Response, ZERO, classify_twist, measured_twist


CARGO = "blade_transport"
VEHICLES = ("robot1", "robot2")
RESIDUAL_LIMIT = .12  # Maximum correction at either mount, in m/s.
PAIR_MODES = (ACKERMANN, OBLIQUE, SPIN)
MODE_BY_FIELD = {(str(mode),): mode for mode in PAIR_MODES}
MODE_KEY = "motion_mode"
FEEDBACK_MODES_KEY = "motion_modes"


def stamp_ns(header):
    return header.stamp.sec * 10**9 + header.stamp.nsec


def write_stamp(header, stamp):
    header.stamp.sec, header.stamp.nsec = divmod(stamp, 10**9)


def put_pose(message, pose):
    p, q = pose.pos(), pose.rot()
    message.position.x, message.position.y, message.position.z = p.x(), p.y(), p.z()
    message.orientation.x, message.orientation.y = q.x(), q.y()
    message.orientation.z, message.orientation.w = q.z(), q.w()


def turn(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c*x - s*y, s*x + c*y


def project_mounts(offsets, heading, velocities):
    """Fit the two requested world mount velocities to one planar rigid twist."""
    (ax, ay), (bx, by) = velocities
    dx, dy = turn(offsets[1][0] - offsets[0][0], offsets[1][1] - offsets[0][1], heading)
    omega = (dx*(by-ay) - dy*(bx-ax)) / (dx*dx + dy*dy)
    # The blade reference origin is the midpoint of the supports.
    vx, vy = (ax+bx)*.5, (ay+by)*.5
    residual = abs(dx*(bx-ax) + dy*(by-ay)) / (2*math.hypot(dx, dy))
    return vx, vy, omega, residual


def new_poses(observed, offsets, twists, dt, mode=ACKERMANN):
    """Advance the assembly once, then apply the two independently driven yaws."""
    cargo, *cars = observed
    a, b = (car.pos() for car in cars)
    x, y = (a.x()+b.x())*.5, (a.y()+b.y())*.5
    local_dx, local_dy = offsets[1][0]-offsets[0][0], offsets[1][1]-offsets[0][1]
    heading = math.atan2(b.y()-a.y(), b.x()-a.x()) - math.atan2(local_dy, local_dx)
    world_velocities = [turn(*twist[:2], car.rot().yaw())
                        for twist, car in zip(twists, cars, strict=True)]
    vx, vy, omega, residual = project_mounts(offsets, heading, world_velocities)
    if mode == OBLIQUE:
        # Parallel translation permits one shared world velocity and no
        # rotation of the cargo. Report the full correction at a mount.
        omega = 0.
        residual = math.dist(*world_velocities) / 2.
    elif mode == SPIN:
        vx = vy = omega = 0.
        residual = max(math.hypot(*velocity) for velocity in world_velocities)
    # Integrate the cargo twist about the reference midpoint. This keeps the
    # spacing exact even for a turn, where forward Euler would stretch it.
    half = omega * dt * .5
    scale = math.sin(half) / half if half else 1.
    dx, dy = turn(vx, vy, half)
    x, y, heading = x + dt*scale*dx, y + dt*scale*dy, heading + 2*half
    result = [Pose3d(x, y, cargo.pos().z(), 0., 0., heading)]
    for offset, twist, car in zip(offsets, twists, cars, strict=True):
        dx, dy = turn(*offset, heading)
        yaw_delta = 0. if mode == OBLIQUE else dt*twist[2]
        result.append(Pose3d(x+dx, y+dy, car.pos().z(), 0., 0., car.rot().yaw()+yaw_delta))
    return result, residual


def update_pair(responses, now, dt, commands, request):
    if request is not None and any(r.mode != request for r in responses):
        # Both plants hold zero until both have braked, then start the
        # mode-confirmation timer together.
        commands = (ZERO, ZERO)
        if not all(r.can_stop_this_step(dt) for r in responses):
            request = None
    return [response.update(now, dt, command, request)
            for response, command in zip(responses, commands, strict=True)]


class BladeSystem:
    def configure(self, entity, sdf, ecm, _event_manager):
        self.world = World(entity)
        self.world_name = self.world.name(ecm)
        self.settings = PlantSettings(**yaml.safe_load(Path(sdf.get_string("settings")).read_text())["plant"])
        ax, ay, bx, by = map(float, sdf.get_string("mount_offsets").split())
        self.offsets = ((ax, ay), (bx, by))
        if not math.isclose(ax, -bx) or not math.isclose(ay, -by) or not math.hypot(ax, ay):
            raise ValueError("blade mounts must be distinct and symmetric about the reference midpoint")
        self.models = self.links = None
        self.last = self.last_time = None
        self.observation_due = False
        self.response = [Response(self.settings, True) for _ in VEHICLES]
        self.lock = Lock()
        self.pending = {}
        self.command_stamp = -1
        self.command_wall = -math.inf
        self.commands = (ZERO, ZERO)
        self.requested_mode = ACKERMANN
        self.fault = ""
        self.active = False
        self.residual = 0.
        self.transport = Node()
        state_topic = f"/world/{self.world_name}/blade_pair/state"
        self.state = self.transport.advertise(state_topic, Pose_V)
        self.odom = [
            self.transport.advertise(f"/{name}/odom", Odometry) for name in VEHICLES
        ]
        self.motion_state = [
            self.transport.advertise(f"/{name}/motion_state", Int32) for name in VEHICLES
        ]
        for index, name in enumerate(VEHICLES):
            self.transport.subscribe(Twist, f"/{name}/cmd_vel",
                                     lambda message, i=index: self.on_twist(i, message))

    def on_twist(self, index, message):
        stamp = stamp_ns(message.header)
        twist = (message.linear.x, message.linear.y, message.angular.z)
        with self.lock:
            if stamp <= self.command_stamp:
                return
            mode_fields = [tuple(field.value) for field in message.header.data if field.key == MODE_KEY]
            mode = ACKERMANN if not mode_fields else None
            if len(mode_fields) == 1:
                mode = MODE_BY_FIELD.get(mode_fields[0])
            if mode is None or not all(math.isfinite(x) for x in twist):
                self.fault = "invalid_paired_twist_or_motion_mode"
                return
            if twist != ZERO and classify_twist(twist, self.settings.switch_radius)[0] != mode:
                self.fault = "twist_crosses_requested_mode_boundary"
                return
            if ((mode == ACKERMANN and twist[0] < 0.) or (mode == OBLIQUE and twist[2] != 0.)
                    or (mode == SPIN and twist[:2] != (0., 0.))):
                self.fault = "twist_violates_requested_motion_constraint"
                return
            pair = self.pending.setdefault(stamp, {})
            pair[index] = (mode, twist)
            if len(pair) == len(VEHICLES):
                requests = tuple(pair[i] for i in range(len(VEHICLES)))
                if requests[0][0] != requests[1][0]:
                    self.fault = "paired_motion_modes_disagree"
                    return
                self.command_stamp = stamp
                self.command_wall = monotonic()
                self.requested_mode = mode
                self.commands = tuple(twist for _, twist in requests)
                self.pending = {key: val for key, val in self.pending.items() if key > stamp}
            elif len(self.pending) > 8:
                del self.pending[min(self.pending)]

    def find_models(self, ecm):
        by_name = {Model(entity).name(ecm): Model(entity) for entity in self.world.models(ecm)}
        names = (CARGO, *VEHICLES)
        if all(name in by_name and by_name[name].canonical_link(ecm) for name in names):
            self.models = [by_name[name] for name in names]
            self.links = [Link(model.canonical_link(ecm)) for model in self.models]

    def pre_update(self, info, ecm):
        if info.paused:
            return
        if self.models is None:
            self.find_models(ecm)
        now = info.sim_time.total_seconds()
        self.observation_due = self.models is not None and (
            self.last_time is None or now - self.last_time + 1e-8 >= 1. / self.settings.frequency
        )
        if not self.observation_due or self.last is None:
            return
        dt = now - self.last_time
        with self.lock:
            fresh = (self.command_stamp >= 0 and
                     -1e-6 <= now - self.command_stamp*1e-9 <= self.settings.command_timeout and
                     monotonic() - self.command_wall <= self.settings.command_timeout)
            commands = self.commands if fresh and not self.fault else (ZERO, ZERO)
            if self.active and not fresh:
                self.fault = self.fault or "paired_twist_watchdog"
            request = self.requested_mode if fresh and not self.fault else None
            changing = request is not None and any(r.mode != request for r in self.response)
            self.active = fresh and (changing or any(command != ZERO for command in commands))
        twists = update_pair(self.response, now, dt, commands, request)
        if self.response[0].mode != self.response[1].mode:
            with self.lock:
                self.fault = "paired_plant_modes_desynchronized"
            return
        if all(twist == ZERO for twist in twists):
            self.residual = 0.
            return
        updated, self.residual = new_poses(self.last, self.offsets, twists, dt, self.response[0].mode)
        if self.residual > RESIDUAL_LIMIT:
            with self.lock:
                self.fault = "incompatible_mount_velocities"
            return
        for model, target in zip(self.models, updated, strict=True):
            model.set_world_pose_cmd(ecm, target)

    def post_update(self, info, ecm):
        if info.paused or not self.observation_due:
            return
        now = info.sim_time.total_seconds()
        # The cargo canonical link is raised to blade height, but its planar
        # reference matches the model. Use the previous model height for poses.
        measured = [link.world_pose(ecm) for link in self.links]
        if any(p is None for p in measured):
            return
        p = measured[0]
        measured[0] = Pose3d(p.pos().x(), p.pos().y(), 0., 0., 0., p.rot().yaw())
        previous, last_time = self.last, self.last_time
        self.last, self.last_time = measured, now
        stamp = round(now * 1e9)
        state = Pose_V()
        write_stamp(state.header, stamp)
        state.header.data.add(key="constraint_residual_mps", value=[str(self.residual)])
        state.header.data.add(key=FEEDBACK_MODES_KEY, value=[str(r.mode) for r in self.response])
        with self.lock:
            if self.fault:
                state.header.data.add(key="fault", value=[self.fault])
        for name, pose in zip((CARGO, *VEHICLES), measured, strict=True):
            put_pose(state.pose.add(name=name), pose)
        self.state.publish(state)
        for index, (name, pose) in enumerate(zip(VEHICLES, measured[1:], strict=True), start=1):
            odom = Odometry()
            write_stamp(odom.header, stamp)
            odom.header.data.add(key="frame_id", value=["map"])
            odom.header.data.add(key="child_frame_id", value=[f"{name}/base_link"])
            put_pose(odom.pose, pose)
            vx, vy, yaw = measured_twist(previous[index], pose, now-last_time) if previous else ZERO
            odom.twist.linear.x, odom.twist.linear.y, odom.twist.angular.z = vx, vy, yaw
            self.odom[index-1].publish(odom)
            self.motion_state[index-1].publish(Int32(data=self.response[index-1].mode))


def get_system():
    return BladeSystem()
