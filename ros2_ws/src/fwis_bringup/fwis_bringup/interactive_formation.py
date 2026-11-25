"""Map-frame formation placement for the Gazebo-only interactive trial."""

from math import cos, sin

from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import (
    CargoFootprint, FormationCollisionChecker, VehicleShape,
)
from ranger_mini_gz.model_launch import SPAWN_HEIGHT
from ranger_mini_gz.scene_generator import clearance_summary
from ranger_mini_gz.scene_geometry import local_pose


class FormationPlacement:
    def __init__(self, scene, cells, margin):
        from gz.transport13 import Node

        self.scene = scene
        self.transport = Node()
        self.service = f"/world/{scene.name}/set_pose_vector/blocking"
        model = FormationModel([VehicleOffset(i, *offset) for i, offset in
                                zip(scene.vehicle_ids, scene.offsets, strict=True)])
        shapes = dict.fromkeys(scene.vehicle_ids, VehicleShape(*scene.vehicle_size[:2]))
        cargo = CargoFootprint(*scene.cargo.size[:2], *scene.cargo.offset)
        self.checker = FormationCollisionChecker(Grid(cells != 0, scene.resolution, 0., 0.),
                                                  model, shapes, margin, cargo_footprint=cargo)

    def vehicle_poses(self, reference):
        return {i: local_pose(reference, offset) for i, offset in
                zip(self.scene.vehicle_ids, self.scene.offsets, strict=True)}

    def obstruction(self, reference):
        gaps = clearance_summary(self.scene, "formation", [(reference, None)])
        for name, gap in gaps.items():
            if gap["distance_m"] <= 0.:
                return f"{name} 与 {gap['obstacle']} 重叠"
        report = self.checker.static_check(FormationPose(*reference),
                                           dict.fromkeys(self.scene.vehicle_ids, reference[2]))
        if not report.success:
            return f"{report.reason} · {report.detail}"
        return ""

    def place(self, reference):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_v_pb2 import Pose_V

        request = Pose_V()
        vehicles = [(f"robot{i}", pose, SPAWN_HEIGHT) for i, pose in self.vehicle_poses(reference).items()]
        for name, (x, y, heading), z in [*vehicles, (self.scene.cargo.name, reference, self.scene.cargo.bottom)]:
            pose = request.pose.add(name=name)
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.z, pose.orientation.w = sin(heading / 2), cos(heading / 2)
        received, reply = self.transport.request(self.service, request, Pose_V, Boolean, 750)
        return received and reply.data


def endpoint_markers(start, goal, stamp):
    markers = []
    for index, (value, color) in enumerate(((start, (.12, .86, .55)), (goal, (1., .61, .14)))):
        marker = Marker(ns="interactive_endpoint", id=index, type=Marker.ARROW)
        marker.header.frame_id, marker.header.stamp = "map", stamp
        marker.action = Marker.ADD if value is not None else Marker.DELETE
        if value is not None:
            x, y, heading = value
            marker.pose = Pose()
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, .12
            marker.pose.orientation.z, marker.pose.orientation.w = sin(heading / 2), cos(heading / 2)
            marker.scale.x, marker.scale.y, marker.scale.z = .70, .19, .16
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.
        markers.append(marker)
    return MarkerArray(markers=markers)
