from math import pi

from builtin_interfaces.msg import Time
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
import gz.transport13
import pytest
from visualization_msgs.msg import Marker

from fwis_bringup.interactive_formation import FormationPlacement, endpoint_markers
from ranger_mini_gz.scene_generator import load_scene
from ranger_mini_gz.scene_geometry import rasterize


def test_group_start_uses_one_gazebo_request_and_fixed_cargo_offsets(monkeypatch):
    requests = []

    class Transport:
        def request(self, service, payload, request_type, reply_type, timeout):
            assert request_type is Pose_V and reply_type is Boolean and timeout > 0
            requests.append((service, payload))
            return True, Boolean(data=True)

    monkeypatch.setattr(gz.transport13, "Node", Transport)
    scene = load_scene("factory_indoor_01")
    cells = rasterize(scene.obstacles, scene.size, scene.resolution)
    placement = FormationPlacement(scene, cells, .05)
    assert placement.obstruction((.08, .08, 0.))
    assert not placement.obstruction((4.4, 5.6, 0.))
    assert placement.place((4.4, 5.6, pi / 2))
    (service, command), = requests
    assert service == f"/world/{scene.name}/set_pose_vector/blocking"
    poses = {pose.name: pose for pose in command.pose}
    assert set(poses) == {"robot1", "robot2", "robot3", scene.cargo.name}
    for name, xy in {"robot1": (4.4, 6.6), "robot2": (3.4, 5.6), "robot3": (5.4, 5.6),
                     scene.cargo.name: (4.4, 5.6)}.items():
        pose = poses[name]
        assert (pose.position.x, pose.position.y) == pytest.approx(xy)
        assert pose.orientation.z == pytest.approx(pose.orientation.w)
    assert poses[scene.cargo.name].position.z == pytest.approx(scene.cargo.bottom)


def test_rviz_endpoints_have_distinct_ids_and_can_be_cleared():
    stamp = Time(sec=3)
    packet = endpoint_markers((4., 5., 0.), (10., 7., pi / 2), stamp)
    start, goal = packet.markers
    assert start.id != goal.id and start.action == goal.action == Marker.ADD
    assert start.header.frame_id == goal.header.frame_id == "map"
    assert start.header.stamp == goal.header.stamp == stamp
    assert start.color != goal.color
    assert all(marker.action == Marker.DELETE for marker in endpoint_markers(None, None, stamp).markers)
