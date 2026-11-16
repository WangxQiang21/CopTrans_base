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


if __name__ == "__main__":
    unittest.main()