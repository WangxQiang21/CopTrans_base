"""G5-B: the ROS-free receiver core of the virtual playback node.

The node script needs ROS imports, but the verification path itself does not
start a ROS master.  These tests rebuild the raw request from the retained
transport fields and require the planner-published admission/context records
before any virtual envelope is accepted.  Real topic and TF behaviour is
covered separately by ``ros_virtual_a_smoke.py``.
"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(ROOT / "tests"))

import virtual_a_g5_support as g5  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    GoalPolicy,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    PlaybackValidationError,
    PlaybackVehicleSpec,
)
from visualization_msgs.msg import Marker  # noqa: E402

_NODE_PATH = ROOT / "catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py"
_SPEC = importlib.util.spec_from_file_location(
    "g5_formation_playback_node", _NODE_PATH)
_NODE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_NODE)


def raw_document(request):
    centers = request.model.centers(request.start_qf)
    goals = request.model.centers(request.goal_qf)
    free = request.goal_policy == GoalPolicy.FREE
    vehicles = []
    for offset in request.model.offsets:
        shape = request.vehicle_shapes[offset.vehicle_id]
        vehicles.append({
            "vehicle_id": offset.vehicle_id,
            "offset_x": offset.x,
            "offset_y": offset.y,
            "start_x": centers[offset.vehicle_id][0],
            "start_y": centers[offset.vehicle_id][1],
            "start_theta": request.start_headings[offset.vehicle_id],
            "goal_x": goals[offset.vehicle_id][0],
            "goal_y": goals[offset.vehicle_id][1],
            "goal_theta": request.goal_qf.psi,
            "goal_heading_free": free,
            "body_length_m": shape.length_m,
            "body_width_m": shape.width_m,
        })
    return {
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "formation_start": {
            "x": request.start_qf.x,
            "y": request.start_qf.y,
            "theta": request.start_qf.psi,
        },
        "formation_goal": {
            "x": request.goal_qf.x,
            "y": request.goal_qf.y,
            "theta": request.goal_qf.psi,
        },
        "allow_reverse": False,
        "vehicles": vehicles,
    }


def envelope_message(envelope):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            secs=envelope["trajectory_fields"]["stamp"][0],
            nsecs=envelope["trajectory_fields"]["stamp"][1])),
        plan_id=envelope["plan_id"],
        formation_id=envelope["formation_id"],
        backend=envelope["backend"],
        semantic_version=envelope["semantic_version"],
        model_version=envelope["model_version"],
        model_sha256=envelope["model_sha256"],
        model_json=envelope["model_json"],
        certification=envelope["certification"],
        trajectory=SimpleNamespace())


class _Publisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class VirtualClearPublicationTest(unittest.TestCase):

    def _node(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node.frame_id = "map"
        node.transport_ui = True
        node.publish_tf = False
        node.tf_broadcaster = None
        node._tf_prefix = "formation_cargo_wedge"
        node._frontend_reports = {}
        node._active_plan = 600007
        node._hold_reason = "PLANNING"
        node._clear_markers_pending = False
        node.trajectory = None
        node.hold = ("cargo_wedge", (0.0, 0.0, 0.0), (
            (PlaybackVehicleSpec(1, 1.0, 0.0, 0.738, 0.5),
             1.0, 0.0, 0.0),
            (PlaybackVehicleSpec(2, 0.0, 1.0, 0.738, 0.5),
             0.0, 1.0, 0.0),
            (PlaybackVehicleSpec(3, 0.0, -1.0, 0.738, 0.5),
             0.0, -1.0, 0.0)))
        node.marker_pub = _Publisher()
        node.status_pub = _Publisher()
        node._playback_model_sha256 = "a" * 64
        node.backend = "virtual_rigid_a"
        node._playback_semantic_version = "virtual_rigid_a.playback.v1"
        return node

    def test_clear_and_hold_markers_are_one_atomic_message(self):
        node = self._node()
        with mock.patch.object(_NODE.rospy.Time, "now",
                               return_value=_NODE.rospy.Time(0, 0)):
            node._clear("new experiment 600007 admitted")
        self.assertEqual(len(node.marker_pub.messages), 1)
        markers = node.marker_pub.messages[0].markers
        self.assertGreater(len(markers), 1)
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        # The hold replacements follow the deletion in the same array, so a
        # queue_size=1 publisher cannot drop the deletion on its own.
        self.assertEqual(len(node.status_pub.messages), 2)
        status = json.loads(node.status_pub.messages[0].data)
        self.assertTrue(status["cleared"])
        self.assertEqual(status["reason"], "new experiment 600007 admitted")
        hold = json.loads(node.status_pub.messages[1].data)
        self.assertEqual(hold["state"], "STATIC")
        self.assertTrue(hold["cleared"])

    def test_first_trajectory_frame_atomically_clears_the_hold(self):
        node = self._node()
        node.publish_tf = False
        node.options = SimpleNamespace()
        node.trajectory = SimpleNamespace(plan_id=600007, stamp=(0, 0))
        node._last_state = "STATIC"
        node._clear_markers_pending = True
        node._status_for_trajectory = lambda: None
        node._marker_array = lambda specs, stamp: list(specs)
        marker_spec = SimpleNamespace(action="ADD")
        frame = SimpleNamespace(
            state="PLAYING", terminal=False, elapsed_s=0.0,
            clamped_elapsed_s=0.0, fraction=0.0)
        with mock.patch.object(_NODE, "frame_at", return_value=frame), \
                mock.patch.object(
                    _NODE, "build_markers",
                    return_value=([marker_spec], ["vehicle"], "STATIC",
                                  False)), \
                mock.patch.object(
                    _NODE, "status_document",
                    return_value={"state": "PLAYING"}), \
                mock.patch.object(_NODE.rospy.Time, "now",
                                  return_value=_NODE.rospy.Time(0, 0)):
            node._publish_tick()
        self.assertEqual(len(node.marker_pub.messages), 1)
        published = node.marker_pub.messages[0]
        self.assertEqual(published[0].action, "DELETEALL")
        self.assertIs(published[-1], marker_spec)
        self.assertFalse(node._clear_markers_pending)


class VirtualReceiverTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.free_case = g5.solved("S2", "free")
        cls.case = g5.solved("S2", "straight")
        cls.request = cls.case["map_request"].request
        cls.envelope = g5.envelope_for(cls.case)
        cls.fields = cls.envelope["trajectory_fields"]
        cls.context = cls.case["context"]
        cls.grid_sha = cls.case["map_request"].grid_sha256
        scene = cls.case["scene"]
        cls.initial = g5.reset_context(
            scene, context_id=cls.request.initial.context_id)

    def _job(self, **overrides):
        job = {
            "schema": "formation_job/v1",
            "state": "PLANNING",
            "backend": "virtual_rigid_a",
            "plan_id": self.request.plan_id,
            "formation_id": self.request.formation_id,
            "request_sha256": request_fingerprint(self.request),
            "planning_context_sha256": self.case["context_sha256"],
            "grid_sha256": self.grid_sha,
            "virtual_a_goal_policy": self.request.goal_policy.value.lower(),
            "planning_context": copy.deepcopy(self.context),
            "initial_context": self.initial.as_dict(),
            "initial_compatible": True,
        }
        job.update(overrides)
        return job

    def _node(self, *, raw="default", job="default"):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node._lock = threading.RLock()
        node._raw_requests = {}
        node._virtual_jobs = {}
        if raw != "default":
            node._raw_requests[self.request.plan_id] = raw
        if job != "default":
            node._virtual_jobs[self.request.plan_id] = job
        return node

    def _verify(self, *, raw="default", job="default", envelope=None,
                fields=None):
        node = self._node(
            raw=raw_document(self.request) if raw == "default" else raw,
            job=self._job() if job == "default" else job)
        message = envelope_message(
            self.envelope if envelope is None else envelope)
        return node._verify_virtual_envelope(
            message, self.fields if fields is None else fields)

    def test_valid_raw_request_and_admission_are_accepted(self):
        trajectory, plan, digest = self._verify()
        self.assertEqual(trajectory.plan_id, self.request.plan_id)
        self.assertEqual(trajectory.formation_id, self.request.formation_id)
        self.assertEqual(trajectory.backend, "virtual_rigid_a")
        self.assertEqual(trajectory.model_sha256, digest)
        self.assertIs(trajectory.virtual_a_plan, plan)

    def test_missing_raw_request_is_rejected(self):
        with self.assertRaises(PlaybackValidationError):
            self._verify(raw={})

    def test_missing_admitted_job_is_rejected(self):
        with self.assertRaises(PlaybackValidationError):
            self._verify(job={})

    def test_raw_start_must_match_the_confirmed_reset(self):
        raw = raw_document(self.request)
        raw["formation_start"]["x"] += 1e-3
        with self.assertRaises(PlaybackValidationError):
            self._verify(raw=raw)

    def test_job_request_digest_must_match_the_raw_request(self):
        job = self._job(request_sha256="0" * 64)
        with self.assertRaises(PlaybackValidationError):
            self._verify(job=job)

    def test_planning_context_must_name_the_raw_request(self):
        context = copy.deepcopy(self.context)
        context["request_sha256"] = "1" * 64
        job = self._job(planning_context=context)
        with self.assertRaises(PlaybackValidationError):
            self._verify(job=job)

    def test_goal_policy_must_match_the_raw_flags(self):
        job = self._job(virtual_a_goal_policy="free")
        with self.assertRaises(PlaybackValidationError):
            self._verify(job=job)

    def test_incompatible_initial_context_is_rejected(self):
        initial = self.initial.as_dict()
        initial["compatible"] = False
        initial["detail"] = "not stationary"
        job = self._job(initial_context=initial)
        with self.assertRaises(PlaybackValidationError):
            self._verify(job=job)

    def test_zero_grid_digest_is_rejected(self):
        job = self._job(grid_sha256=None)
        with self.assertRaises(PlaybackValidationError):
            self._verify(job=job)

    def test_verified_terminal_heading_must_match_the_raw_start(self):
        initial = self.initial.as_dict()
        initial["source"] = "verified_terminal"
        request_sha = request_fingerprint(self.request)
        job = self._job(initial_context=initial, request_sha256=request_sha)
        # The raw request still names the reset context; rebuilding with the
        # terminal source must fail before any trajectory is accepted.
        with self.assertRaises(PlaybackValidationError):
            self._verify(job=job)

    def test_tampered_model_is_rejected_by_the_verifier(self):
        envelope = dict(self.envelope)
        envelope["model_json"] = self.envelope["model_json"].replace(
            "1.0", "1.5", 1)
        with self.assertRaises(PlaybackValidationError):
            self._verify(envelope=envelope)

    def test_busy_status_does_not_erase_the_playing_certification(self):
        node = self._node()
        node.transport_ui = False
        node._active_plan = self.request.plan_id
        node.optimizer_status = None
        node._optimizer_status_by_plan = {}
        node.quality_report = None
        node.trajectory = SimpleNamespace(
            plan_id=self.request.plan_id,
            formation_id=self.request.formation_id,
            model_sha256=self.envelope["model_sha256"],
            backend="virtual_rigid_a")
        accepted = {
            "plan_id": self.request.plan_id,
            "formation_id": self.request.formation_id,
            "backend": "virtual_rigid_a",
            "model_sha256": self.envelope["model_sha256"],
            "status": "FEASIBLE",
            "executable": True,
            "feasible": True,
            "trajectory_published": True,
        }
        busy = dict(accepted, plan_id=self.request.plan_id + 500000,
                    status="BUSY_REJECTED", executable=False, feasible=False,
                    trajectory_published=False)
        busy_reset = dict(
            accepted, admitted=False, busy_rejected=True,
            reset_consumed=False, formation_id=None,
            status="REJECTED", executable=False, feasible=False,
            trajectory_published=False)
        node.on_optimizer_status(
            SimpleNamespace(data=json.dumps(accepted)))
        node.on_optimizer_status(SimpleNamespace(data=json.dumps(busy)))
        node.on_optimizer_status(
            SimpleNamespace(data=json.dumps(busy_reset)))
        report = node._status_for_trajectory()
        self.assertIsNotNone(report)
        self.assertEqual(report["plan_id"], self.request.plan_id)
        self.assertEqual(report["status"], "FEASIBLE")

    def test_uniform_envelope_cannot_use_the_virtual_receiver(self):
        envelope = dict(self.envelope)
        envelope["backend"] = "uniform_v02"
        with self.assertRaises(PlaybackValidationError):
            self._verify(envelope=envelope)


class VirtualTransportPreviewTest(unittest.TestCase):
    """A raw virtual request must end the goal-bridge transport preview.

    The goal bridge latches ``/formation/display_pose`` at a manual reset,
    which sets ``_previewing`` and makes ``on_playback_plan`` ignore any
    envelope.  Raw virtual request receipt is not admission and must not
    clear the replay/hold/diagnostics, but it must leave preview mode;
    otherwise the atomic envelope for the admitted experiment is dropped
    silently.  A busy request for another admitted plan keeps the preview,
    matching the pre-existing non-virtual guard ordering.
    """

    def _node(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node._lock = threading.RLock()
        node.backend = "virtual_rigid_a"
        node._job_state = "IDLE"
        node._job_state_plan = None
        node._previewing = True
        node.trajectory = None
        node._active_plan = None
        node._hold_reason = "START"
        node.hold = None
        node._clear_markers_pending = False
        node._record_raw_request = lambda message: None
        return node

    def test_idle_virtual_request_ends_the_preview(self):
        node = self._node()
        node.on_request(SimpleNamespace(plan_id=600000))
        self.assertFalse(node._previewing)
        self.assertIsNone(node.trajectory)
        self.assertEqual(node._hold_reason, "START")

    def test_matching_admitted_virtual_request_ends_the_preview(self):
        node = self._node()
        node._job_state = "PLANNING"
        node._job_state_plan = 600000
        node.on_request(SimpleNamespace(plan_id=600000))
        self.assertFalse(node._previewing)

    def test_busy_virtual_request_keeps_the_preview(self):
        node = self._node()
        node._job_state = "PLANNING"
        node._job_state_plan = 600001
        node.on_request(SimpleNamespace(plan_id=600000))
        self.assertTrue(node._previewing)


class VehiclePathAndResetTest(unittest.TestCase):
    """RViz Path topics and the 2D Pose Estimate reset behaviour."""

    @classmethod
    def setUpClass(cls):
        cls.case = g5.solved("S2", "straight")
        cls.envelope = g5.envelope_for(cls.case)

    def _receiver_node(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node._lock = threading.RLock()
        node._raw_requests = {}
        node._virtual_jobs = {}
        request = self.case["map_request"].request
        node._raw_requests[request.plan_id] = raw_document(request)
        node._virtual_jobs[request.plan_id] = {
            "schema": "formation_job/v1",
            "state": "PLANNING",
            "backend": "virtual_rigid_a",
            "plan_id": request.plan_id,
            "formation_id": request.formation_id,
            "request_sha256": request_fingerprint(request),
            "planning_context_sha256": self.case["context_sha256"],
            "grid_sha256": self.case["map_request"].grid_sha256,
            "virtual_a_goal_policy": request.goal_policy.value.lower(),
            "planning_context": copy.deepcopy(self.case["context"]),
            "initial_context": g5.reset_context(
                self.case["scene"],
                context_id=request.initial.context_id).as_dict(),
            "initial_compatible": True,
        }
        return node

    def test_full_path_is_published_once_as_a_path_message(self):
        node = self._receiver_node()
        message = envelope_message(self.envelope)
        trajectory, _plan, _digest = node._verify_virtual_envelope(
            message, self.envelope["trajectory_fields"])
        publishers = {vehicle_id: _Publisher()
                      for vehicle_id in trajectory.points}
        node.path_pubs = publishers
        node._publish_paths(trajectory)
        stamp = trajectory.stamp
        for vehicle_id, publisher in publishers.items():
            self.assertEqual(len(publisher.messages), 1)
            path = publisher.messages[0]
            self.assertEqual(path.header.frame_id, "map")
            self.assertEqual(
                (path.header.stamp.secs, path.header.stamp.nsecs), stamp)
            samples = trajectory.points[vehicle_id]
            self.assertEqual(len(path.poses), len(samples))
            for pose, sample in zip(path.poses, samples):
                self.assertEqual(pose.header.frame_id, "map")
                self.assertAlmostEqual(
                    pose.pose.position.x, sample.center_x, places=12)
                self.assertAlmostEqual(
                    pose.pose.position.y, sample.center_y, places=12)
                quaternion = pose.pose.orientation
                yaw = math.atan2(
                    2 * (quaternion.w * quaternion.z +
                         quaternion.x * quaternion.y),
                    1 - 2 * (quaternion.y * quaternion.y +
                             quaternion.z * quaternion.z))
                self.assertLess(abs(math.atan2(
                    math.sin(yaw - sample.theta),
                    math.cos(yaw - sample.theta))), 1e-9)

    def test_diagnostic_paths_clear_stale_routes_before_replacement(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node.frame_id = "map"
        node.guide_path_pubs = {
            "/formation/path/astar_raw_0": _Publisher(),
            "/formation/path/astar_raw_2": _Publisher(),
            "/formation/path/portal_guide_0": _Publisher(),
        }
        node.candidate_path_pubs = {
            "/formation/path/f_candidate": _Publisher(),
            "/formation/path/candidate_vehicle_1": _Publisher(),
        }
        stamp = _NODE.rospy.Time(3, 4)
        with mock.patch.object(
                _NODE.rospy.Time, "now", return_value=stamp):
            node._publish_guide_paths({
                ("astar_raw", 0): ((0.0, 1.0), (2.0, 3.0)),
                ("portal_guide", 0): ((4.0, 5.0),),
            }, stamp)
            node._publish_candidate_paths({
                "f_candidate": ((6.0, 7.0), (8.0, 9.0)),
                ("candidate_vehicle", 1): ((10.0, 11.0),),
            }, stamp)
        astar = node.guide_path_pubs[
            "/formation/path/astar_raw_0"].messages[-1]
        self.assertEqual((astar.header.stamp.secs,
                          astar.header.stamp.nsecs), (3, 4))
        self.assertEqual(astar.header.frame_id, "map")
        self.assertEqual(len(astar.poses), 2)
        self.assertEqual(
            (astar.poses[0].pose.position.x, astar.poses[0].pose.position.y),
            (0.0, 1.0))
        self.assertEqual(astar.poses[0].pose.orientation.w, 1.0)
        self.assertEqual(
            len(node.guide_path_pubs[
                "/formation/path/portal_guide_0"].messages[-1].poses), 1)
        # A shorter payload must empty every route topic not present now.
        with mock.patch.object(
                _NODE.rospy.Time, "now", return_value=stamp):
            node._publish_guide_paths({
                ("astar_raw", 0): ((0.0, 1.0),),
            }, stamp)
            node._publish_candidate_paths({"f_candidate": ((6.0, 7.0),)},
                                          stamp)
        self.assertEqual(
            len(node.guide_path_pubs[
                "/formation/path/portal_guide_0"].messages[-1].poses), 0)
        self.assertEqual(
            len(node.guide_path_pubs[
                "/formation/path/astar_raw_2"].messages[-1].poses), 0)
        self.assertEqual(
            len(node.guide_path_pubs[
                "/formation/path/astar_raw_0"].messages[-1].poses), 1)
        self.assertEqual(
            len(node.candidate_path_pubs[
                "/formation/path/candidate_vehicle_1"].messages[-1].poses),
            0)
        self.assertEqual(
            len(node.candidate_path_pubs[
                "/formation/path/f_candidate"].messages[-1].poses), 1)

    def test_2d_pose_estimate_clears_every_old_display_layer(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node._lock = threading.RLock()
        node.frame_id = "map"
        node.transport_ui = True
        node.backend = "virtual_rigid_a"
        node.trajectory = SimpleNamespace(plan_id=600007)
        node._active_plan = 600007
        node._previewing = False
        node._clear_markers_pending = False
        node._hold_reason = "FAILED"
        node.contract = SimpleNamespace(
            formation_id="cargo_wedge",
            body_length_m=0.738,
            body_width_m=0.5,
            vehicle_ids=(1, 2, 3),
            offsets=((1.0, 0.0), (0.0, 1.0), (0.0, -1.0)),
            headings_for_policy=lambda yaw: {1: yaw, 2: yaw, 3: yaw})
        node.corridor_pub = _Publisher()
        node.guide_pub = _Publisher()
        node.candidate_pub = _Publisher()
        node.path_pubs = {1: _Publisher(), 2: _Publisher(), 3: _Publisher()}
        node.guide_path_pubs = {
            "/formation/path/astar_raw_0": _Publisher(),
            "/formation/path/portal_guide_0": _Publisher(),
        }
        node.candidate_path_pubs = {
            "/formation/path/f_candidate": _Publisher(),
            "/formation/path/candidate_vehicle_1": _Publisher(),
            "/formation/path/candidate_vehicle_2": _Publisher(),
            "/formation/path/candidate_vehicle_3": _Publisher(),
        }
        hold_calls = []
        node._publish_hold = lambda clear=False, stamp=None: hold_calls.append(
            clear)
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        with mock.patch.object(
                _NODE.rospy.Time, "now",
                return_value=_NODE.rospy.Time(0, 0)):
            node.on_preview(message)
        self.assertTrue(node._previewing)
        self.assertIsNone(node.trajectory)
        self.assertIsNone(node._active_plan)
        self.assertEqual(node._hold_reason, "START")
        self.assertEqual(hold_calls, [True])
        for publisher in (node.corridor_pub, node.guide_pub,
                          node.candidate_pub):
            self.assertEqual(len(publisher.messages), 1)
            marker = publisher.messages[0].markers[0]
            self.assertEqual(marker.action, Marker.DELETEALL)
        for registry in (node.path_pubs, node.guide_path_pubs,
                         node.candidate_path_pubs):
            for publisher in registry.values():
                self.assertEqual(len(publisher.messages), 1)
                path = publisher.messages[0]
                self.assertEqual(path.header.frame_id, "map")
                self.assertEqual(len(path.poses), 0)


if __name__ == "__main__":
    unittest.main()