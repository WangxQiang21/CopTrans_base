"""Feed the standard Ranger dashboards and models from the dual-blade Gazebo run."""

import argparse
from array import array
from collections import deque
import json
import math
from pathlib import Path
from threading import Lock
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GazeboNode, SubscribeOptions
from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
import shapely as sg
from std_msgs.msg import ColorRGBA, Float64, Int32, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import xacro

from ranger_mini_gz.rviz_layout import rviz_config
from ranger_mini_gz.scene_generator import ALL_BLADE_SCENES, installed_paths, load_scene, read_map, with_trial_road


NAMES = ("blade_transport", "robot1", "robot2")
TELEMETRY = ("/cargo", "/ranger1", "/ranger2")
COLORS = ((.36, .56, .61), (36/255, 127/255, 176/255), (198/255, 103/255, 33/255))


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array(((c, s), (-s, c))) + pose[:2]


def time_msg(ns):
    sec, nsec = divmod(ns, 10**9)
    return Time(sec=sec, nanosec=nsec)


def yaw_quaternion(q, yaw):
    q.z, q.w = math.sin(yaw/2), math.cos(yaw/2)


def pose_values(pose, planar, z):
    pose.position.x, pose.position.y = map(float, planar[:2])
    pose.position.z = z
    yaw_quaternion(pose.orientation, planar[2])


def marker(kind, group, index, color, alpha=1.):
    shape = Marker(type=kind, ns=group, id=index, action=Marker.ADD)
    shape.header.frame_id = "map"
    shape.pose.orientation.w = 1.
    shape.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=alpha)
    return shape


def points(values, height):
    return [Point(x=float(x), y=float(y), z=height) for x, y in values]


def line(values, group, index, color, width, height, alpha=1.):
    shape = marker(Marker.LINE_STRIP, group, index, color, alpha)
    shape.scale.x = width
    shape.points = points(values, height)
    return shape


def grid_message(path):
    metadata, occupancy = read_map(path)
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = float(metadata["resolution"])
    message.info.height, message.info.width = occupancy.shape
    origin = metadata["origin"]
    message.info.origin.position.x, message.info.origin.position.y = map(float, origin[:2])
    yaw_quaternion(message.info.origin.orientation, origin[2])
    message.data = array("b", occupancy.tobytes())
    return message


def read_plan(path):
    with np.load(path, allow_pickle=False) as data:
        cargo, vehicles = data["cargo"], data["vehicles"]
        count = len(cargo)
        if count < 2 or cargo.shape != (count, 3) or vehicles.shape != (count, 2, 3):
            raise ValueError("blade RViz requires matched cargo and two Ranger pose sequences")
        plan = np.concatenate((cargo[:, None], vehicles), axis=1)
        if not np.isfinite(plan).all():
            raise ValueError("nonfinite blade plan")
    return plan


def reference_path(values):
    path = PathMsg()
    path.header.frame_id = "map"
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(values[:, :2], axis=0), axis=1))]
    samples = np.arange(0., distance[-1], .10)
    indices = np.unique(np.r_[np.searchsorted(distance, samples),len(values)-1])
    for pose in values[indices]:
        row = PoseStamped(header=path.header)
        pose_values(row.pose, pose, 0.)
        path.poses.append(row)
    return path


def planned_envelopes(scene, plan):
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(plan[:, 0, :2], axis=0), axis=1))]
    indices = np.unique(np.clip(np.searchsorted(distance, np.linspace(0, distance[-1], 18)),
                                0, len(distance)-1))
    length, width = scene.vehicle_size[:2]
    chassis = .5*np.array(((-length,-width), (length,-width), (length,width), (-length,width)))
    outlines = (scene.cargo.outline, chassis, chassis)
    markers = []
    for slot, (shape, color) in enumerate(zip(outlines,COLORS,strict=True)):
        polygon = np.vstack((shape, shape[0]))
        for index in indices:
            markers.append(line(transform(polygon, plan[index, slot]), "cargo_footprint",
                                int(index)*3+slot, color, .085, .19, .68))
    for i, gate in enumerate(scene.experiments["blade_pair"]["ordered_gates"], 1):
        x0, x1 = gate["x"]
        y0, y1 = gate["y"]
        markers.append(line([(x0,y0), (x1,y0), (x1,y1), (x0,y1), (x0,y0)],
                            "required_gates", i, (.69,.49,.20), .095, .20, .9))
    return MarkerArray(markers=markers)


def endpoint_markers(plan):
    values = []
    for i, (pose,color) in enumerate(zip(plan[[0,-1],0], ((.25,.66,.41),(.86,.36,.29)),strict=True)):
        shape = marker(Marker.ARROW, "formation_endpoints", i, color)
        pose_values(shape.pose, pose, .25)
        shape.scale.x, shape.scale.y, shape.scale.z = 1.4, .22, .28
        values.append(shape)
    return MarkerArray(markers=values)


def decode_state(message):
    poses = {part.name:part for part in message.pose}
    if not all(name in poses for name in NAMES):
        return None
    values, elevations = [], []
    for name in NAMES:
        p,q = poses[name].position,poses[name].orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        values.append((p.x,p.y,yaw))
        elevations.append(p.z)
    state = np.asarray(values)
    if not np.isfinite(state).all() or not all(map(math.isfinite,elevations)):
        return None
    fields = {entry.key:list(entry.value) for entry in message.header.data if entry.value}
    stamp = message.header.stamp.sec*10**9 + message.header.stamp.nsec
    return stamp,state,elevations,fields


def fixed_model(description, name, color):
    model = xacro.process_file(str(description/"urdf/ranger_mini.urdf.xacro"),
                                mappings={"shell_color":color,
                                          "mesh_uri":"package://ranger_mini_description/meshes/painted"})
    fixed = []
    for joint in ElementTree.fromstring(model.toxml()).findall("joint"):
        if joint.get("type") != "fixed":
            continue
        link_from = joint.find("parent").attrib["link"]
        link_to = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        xyz = tuple(map(float, origin.get("xyz","0 0 0").split())) if origin is not None else (0.,0.,0.)
        rpy = tuple(map(float, origin.get("rpy","0 0 0").split())) if origin is not None else (0.,0.,0.)
        roll,pitch,yaw = rpy
        cr,cp,cy = map(math.cos, (roll/2,pitch/2,yaw/2))
        sr,sp,sy = map(math.sin, (roll/2,pitch/2,yaw/2))
        joint_tf = TransformStamped(child_frame_id=f"{name}/{link_to}")
        joint_tf.header.frame_id = f"{name}/{link_from}"
        tf = joint_tf.transform
        tf.translation.x,tf.translation.y,tf.translation.z = xyz
        tf.rotation.x = sr*cp*cy-cr*sp*sy
        tf.rotation.y = cr*sp*cy+sr*cp*sy
        tf.rotation.z = cr*cp*sy-sr*sp*cy
        tf.rotation.w = cr*cp*cy+sr*sp*sy
        fixed.append(joint_tf)
    return model.toxml(),fixed


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError,json.JSONDecodeError):
        return {}


class BladeView(Node):
    def __init__(self,scene,map_path,plan,description,status_path,plan_report):
        super().__init__("coptrans_blade_rviz")
        self.scene,self.status_path,self.report = scene,status_path,plan_report
        self.experiment_id = status_path.parent.name
        self.plan_id = 1
        self.offsets = np.asarray(scene.offsets)
        self.outline = np.vstack((scene.cargo.outline,scene.cargo.outline[0]))
        triangles = sg.constrained_delaunay_triangles(sg.Polygon(scene.cargo.outline))
        self.triangles = np.concatenate([np.asarray(p.exterior.coords)[:3] for p in sg.get_parts(triangles)])
        self.last,self.latest,self.tracking = None,None,None
        self.observations = deque(maxlen=60)
        self.traced,self.last_error_stamp = [],0
        self.lock = Lock()
        self.transport = GazeboNode()
        options = SubscribeOptions()
        options.msgs_per_sec = 30
        topic = f"/world/{scene.name}/blade_pair"
        self.transport.subscribe(Pose_V,topic+"/state",self.receive_state,options)
        self.transport.subscribe(Pose_V,topic+"/tracking_reference",self.receive_reference,options)

        latched = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid,"/map",latched)
        self.marker_pub = self.create_publisher(MarkerArray,"/scene/markers",latched)
        self.end_pub = self.create_publisher(MarkerArray,"/formation/interactive_endpoints",latched)
        self.envelope_pub = self.create_publisher(MarkerArray,"/formation/cargo_footprint_trace",latched)
        self.solver_pub = self.create_publisher(String,"/formation/uniform_progress",latched)
        self.status_pub = self.create_publisher(String,"/formation/interactive_status",latched)
        self.clock_pub = self.create_publisher(Clock,"/clock",qos_profile_sensor_data)
        self.spacing_pub = self.create_publisher(Float64,"/formation/spacing_error",qos_profile_sensor_data)
        self.odom = [self.create_publisher(Odometry, prefix+"/odom",qos_profile_sensor_data) for prefix in TELEMETRY]
        self.mode = [self.create_publisher(Int32, prefix+"/motion_state",qos_profile_sensor_data)
                     for prefix in TELEMETRY[1:]]
        self.controller = [self.create_publisher(String,prefix+"/controller_status",10) for prefix in TELEMETRY[1:]]
        self.dynamic_tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.models = []
        self.paths = []
        fixed = []
        for i,(name,color) in enumerate(zip(NAMES[1:],("white","orange"),strict=True),1):
            model, transforms = fixed_model(description,name,color)
            fixed.extend(transforms)
            pub = self.create_publisher(String,f"/{name}/robot_description",latched)
            self.models.append(pub)
            pub.publish(String(data=model))
            path = self.create_publisher(PathMsg,f"/formation/path/vehicle_{i}",latched)
            self.paths.append(path)
            path.publish(reference_path(plan[:,i]))
        self.static_tf.sendTransform(fixed)
        self.map_pub.publish(grid_message(map_path))
        self.end_pub.publish(endpoint_markers(plan))
        self.envelope_pub.publish(planned_envelopes(scene,plan))
        self.publish_status(0,"")
        self.create_timer(.10,self.publish_scene)

    def receive_state(self,message):
        state = decode_state(message)
        if state is not None:
            with self.lock:
                if self.latest is None or self.latest[0] < state[0]:
                    self.latest = state
                    self.observations.append((state[0],state[1]))

    def receive_reference(self,message):
        state = decode_state(message)
        if state is not None:
            with self.lock:
                if self.tracking is None or self.tracking[0] < state[0] or state[3].get("phase") == ["COMPLETE"]:
                    self.tracking = state

    def publish_status(self, elapsed, fault, tracking=False):
        report = read_json(self.status_path)
        phase = "transport_executing" if tracking else "waiting"
        detail = "使用本次离线认证的起终点和规划轨迹"
        if report.get("status") in ("passed","failed"):
            phase = "finished" if report["status"] == "passed" else "failed"
            fault = report.get("reason","") if phase == "failed" else ""
        elif fault:
            phase = "failed"
        status = dict(phase=phase,solver_outcome="success",experiment_id=self.experiment_id,
                      plan_id=self.plan_id,vehicle_count=2,admitted=2 if self.last else 0,
                      complete=2 if phase == "finished" else 0,transport_elapsed_s=elapsed,
                      solver_elapsed_s=self.report.get("wall_s",0.),reason=fault,detail=detail)
        self.status_pub.publish(String(data=json.dumps(status)))
        self.solver_pub.publish(String(data=json.dumps(dict(plan_id=self.plan_id,stage="certified",
            actual_backend=self.report.get("planner","blade_pair"),model_schema="RigidBladePair",
            solver_build=self.report.get("source_sha256",""),speed_profile="dual"))))

    def publish_model(self, observed):
        cargo = observed[0]
        height = self.scene.cargo.bottom + self.scene.cargo.size[2]/2
        filled = marker(Marker.TRIANGLE_LIST,"cargo",0,COLORS[0],.34)
        filled.scale.x = filled.scale.y = filled.scale.z = 1.
        filled.points = points(transform(self.triangles,cargo),height)
        markers = [filled,line(transform(self.outline,cargo),"blade_outline",0,COLORS[0],.12,height+.01)]
        for i, xy in enumerate(transform(self.offsets,cargo),1):
            mount = marker(Marker.CYLINDER,"cargo_mounts",i,(.72,.75,.77))
            mount.pose.position = points([xy],self.scene.cargo.bottom-self.scene.cargo.mount_height/2)[0]
            mount.scale.x = mount.scale.y = self.scene.cargo.mount_diameter
            mount.scale.z = self.scene.cargo.mount_height
            markers.append(mount)
        if not self.traced or np.max(np.linalg.norm(observed[:,:2]-self.traced[-1],axis=1)) >= .13:
            self.traced.append(observed[:,:2].copy())
        if len(self.traced)>1:
            trace = np.asarray(self.traced)
            for i,color in enumerate(COLORS):
                markers.append(line(trace[:,i],"observed_trace",i,color,.12,.34,.85))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def publish_feedback(self, state):
        stamp,observed,elevations,fields = state
        now = time_msg(stamp)
        velocity = np.zeros((3,3))
        if self.last is not None:
            dt = 1e-9*(stamp-self.last[0])
            delta = observed-self.last[1]
            for i, pose in enumerate(observed):
                body_delta = transform([delta[i,:2]],(0.,0.,-pose[2]))[0]
                velocity[i] = (*body_delta/dt,math.remainder(delta[i,2],math.tau)/dt)
        frames = []
        for i, (publisher,pose,z,twist) in enumerate(zip(self.odom,observed,elevations,velocity,strict=True)):
            msg = Odometry()
            msg.header.frame_id,msg.header.stamp = "map",now
            msg.child_frame_id = "cargo" if i==0 else f"robot{i}/base_link"
            pose_values(msg.pose.pose,pose,z)
            msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z = map(float,twist)
            publisher.publish(msg)
            if i:
                frame = TransformStamped(child_frame_id=msg.child_frame_id,header=msg.header)
                frame.transform.translation.x,frame.transform.translation.y = map(float,pose[:2])
                frame.transform.translation.z = z
                yaw_quaternion(frame.transform.rotation,pose[2])
                frames.append(frame)
        self.dynamic_tf.sendTransform(frames)
        self.clock_pub.publish(Clock(clock=now))
        modes = fields.get("motion_modes",("0","0"))
        for publisher,mode in zip(self.mode,modes):
            publisher.publish(Int32(data=int(mode) if mode in ("0","1","2") else -1))
        error = abs(float(np.linalg.norm(observed[2,:2]-observed[1,:2]) -
                          np.linalg.norm(self.offsets[1]-self.offsets[0])))
        self.spacing_pub.publish(Float64(data=error))
        self.publish_model(observed)

    def publish_errors(self, reference):
        if reference is None:
            return 0.
        stamp,desired,_,fields = reference
        try:
            epoch = int(fields["trajectory_stamp_ns"][0])
        except (KeyError,TypeError,ValueError):
            return 0.
        errors = fields.get("body_error",[])
        if len(errors) != 9:
            with self.lock:
                observed = next((p for t,p in reversed(self.observations) if t==stamp),None)
            if observed is not None:
                errors = []
                for target,actual in zip(desired,observed,strict=True):
                    heading = -actual[2]
                    xy = transform([target[:2]-actual[:2]],(0.,0.,heading))[0]
                    errors.extend((*xy,math.remainder(target[2]-actual[2],math.tau)))
        phase = fields.get("phase",["TRACKING"])[0]
        if len(errors)==9 and self.last_error_stamp<stamp:
            for i, pub in enumerate(self.controller,1):
                report = dict(kind="execution",plan_id=self.plan_id,trajectory_stamp_ns=epoch,
                              stamp=stamp*1e-9,phase=phase,body_error=list(map(float,errors[3*i:3*i+3])))
                pub.publish(String(data=json.dumps(report)))
            self.last_error_stamp = stamp
        return max(0.,(stamp-epoch)*1e-9)

    def publish_scene(self):
        with self.lock:
            state,reference = self.latest,self.tracking
        if state is None:
            return
        if self.last is None or self.last[0] < state[0]:
            self.publish_feedback(state)
            self.last = state
        self.publish_status(self.publish_errors(reference),state[3].get("fault",[""])[0],
                            tracking=reference is not None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan",type=Path)
    parser.add_argument("map",type=Path)
    parser.add_argument("rviz_config",type=Path)
    parser.add_argument("--scene",choices=ALL_BLADE_SCENES,required=True)
    parser.add_argument("--scene-package-dir",type=Path,required=True)
    parser.add_argument("--tracking-status",type=Path)
    parser.add_argument("--road",type=Path)
    args = parser.parse_args()
    _,common,backends = installed_paths()
    scene = with_trial_road(load_scene(args.scene, paths=(args.scene_package_dir,common,backends)), args.road)
    description = Path(get_package_share_directory("ranger_mini_description"))
    status = args.tracking_status or args.rviz_config.parent/"tracking.json"
    rclpy.init(args=[])
    node = None
    try:
        node = BladeView(scene,args.map,read_plan(args.plan),description,status,read_json(args.plan.parent/"offline.json"))
        rviz_config(description,args.rviz_config.parent,scene,"blade_pair",tracking_dashboard=True,
                    experiment_id=node.experiment_id,config_path=args.rviz_config)
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=="__main__":
    main()
