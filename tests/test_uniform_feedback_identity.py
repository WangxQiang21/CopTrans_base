"""FR05-R2: uniform planner feedback is bound to the published model identity.

The playback status is only authoritative when it belongs to the exact model
the planner published.  A HOLD_END for another digest, formation or backend —
or one missing the identity fields — must not release the busy job or upgrade
the planner status to executable.  These tests drive the real
``UniformPlannerNode.on_playback_status`` without starting ROS.
"""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts"))
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from formation_uniform_planner_node import (  # noqa: E402
    HOLD_END, PLANNING, PLAYING, UniformPlannerNode)


class _ExplicitNull:
    """Marker: put an explicit JSON null into the feedback document."""


NULL = _ExplicitNull()


class FeedbackIdentityTest(unittest.TestCase):

    DIGEST = "a" * 64
    FORMATION = "cargo_wedge"
    PLAN_ID = 700124

    def setUp(self):
        self.events = []

    def _node(self):
        node = UniformPlannerNode.__new__(UniformPlannerNode)
        node._lock = threading.RLock()
        node._state = PLAYING
        node._state_plan = self.PLAN_ID
        node._active_model_sha256 = self.DIGEST
        node._active_formation_id = self.FORMATION
        node._active_status_word = "FEASIBLE"
        node._playback_seen = False
        node._published_wall = 123.0
        node.backend = "uniform_v02"
        node._publish_job_state = lambda *args, **kwargs: self.events.append(
            ("job", node._state))
        node._base_status = lambda *args, **kwargs: dict(
            kwargs, base_args=args)
        node._publish_status = lambda document: self.events.append(
            ("status", document))
        return node

    def _prepublication_node(self):
        """The real state while a request is validated but not yet published."""
        node = self._node()
        node._state = PLANNING
        node._active_model_sha256 = None
        node._active_status_word = None
        node._published_wall = None
        return node

    def _deliver(self, node, **overrides):
        report = {
            "state": "HOLD_END",
            "plan_id": self.PLAN_ID,
            "formation_id": self.FORMATION,
            "backend": "uniform_v02",
            "model_sha256": self.DIGEST,
            "executable": False,
        }
        for name, value in overrides.items():
            if value is NULL:
                report[name] = None
            elif value is None:
                report.pop(name, None)
            else:
                report[name] = value
        with patch.object(
                sys.modules["formation_uniform_planner_node"].rospy,
                "logwarn_throttle"):
            node.on_playback_status(
                SimpleNamespace(data=json.dumps(report)))

    def test_matching_identity_terminal_releases_busy(self):
        node = self._node()
        self._deliver(node)
        self.assertEqual(node._state, HOLD_END)
        statuses = [document for kind, document in self.events
                    if kind == "status"]
        self.assertEqual(len(statuses), 1)
        self.assertTrue(statuses[-1]["feasible"])
        self.assertTrue(statuses[-1]["executable"])
        self.assertTrue(statuses[-1]["published"])

    def test_wrong_digest_terminal_does_not_release_busy(self):
        node = self._node()
        self._deliver(node, model_sha256="b" * 64)
        self.assertEqual(node._state, PLAYING)
        self.assertEqual(self.events, [])

    def test_missing_identity_terminal_does_not_release_busy(self):
        for missing in ("model_sha256", "formation_id", "backend"):
            node = self._node()
            self._deliver(node, **{missing: None})
            self.assertEqual(node._state, PLAYING, missing)
            self.assertEqual(self.events, [], missing)

    def test_wrong_formation_or_backend_terminal_is_ignored(self):
        node = self._node()
        self._deliver(node, formation_id="other_formation")
        self.assertEqual(node._state, PLAYING)
        node = self._node()
        self._deliver(node, backend="legacy")
        self.assertEqual(node._state, PLAYING)

    def test_wrong_digest_hold_start_does_not_confirm_playback(self):
        node = self._node()
        self._deliver(node, state="HOLD_START", model_sha256="b" * 64)
        self.assertFalse(node._playback_seen)
        self.assertNotIn("job", [kind for kind, _ in self.events])

    # --- publication phase boundary (FR05-R2) ---------------------------
    # Before the worker publishes an envelope the job is still PLANNING and
    # ``_active_model_sha256`` is None.  A feedback document that also lacks
    # the digest must not be treated as "None == None" match.

    def _assert_ignored_before_publication(self, node):
        self.assertEqual(node._state, PLANNING)
        self.assertFalse(node._playback_seen)
        self.assertEqual(self.events, [])

    def test_prepublication_missing_digest_terminal_is_ignored(self):
        node = self._prepublication_node()
        self._deliver(node)  # HOLD_END, model_sha256 absent
        self._assert_ignored_before_publication(node)

    def test_prepublication_explicit_null_digest_terminal_is_ignored(self):
        node = self._prepublication_node()
        self._deliver(node, model_sha256=NULL)
        self._assert_ignored_before_publication(node)

    def test_prepublication_missing_digest_start_does_not_confirm(self):
        node = self._prepublication_node()
        self._deliver(node, state="HOLD_START")
        self._assert_ignored_before_publication(node)

    # --- published feedback must keep working ---------------------------
    # The envelope may be published while the node is still PLANNING (first
    # playback confirmation not received yet), and a zero-duration hold can
    # report HOLD_END directly.  Both must release the busy job.

    def test_published_planning_terminal_releases_busy(self):
        node = self._node()
        node._state = PLANNING
        self._deliver(node)
        self.assertEqual(node._state, HOLD_END)
        self.assertTrue(node._playback_seen)
        statuses = [document for kind, document in self.events
                    if kind == "status"]
        self.assertEqual(len(statuses), 1)
        self.assertTrue(statuses[-1]["executable"])
        self.assertTrue(statuses[-1]["published"])

    def test_published_zero_motion_terminal_releases_busy(self):
        node = self._node()
        node._state = PLANNING
        node._active_status_word = "NO_MOTION"
        self._deliver(node)
        self.assertEqual(node._state, HOLD_END)
        self.assertTrue(node._playback_seen)
        statuses = [document for kind, document in self.events
                    if kind == "status"]
        self.assertEqual(len(statuses), 1)
        self.assertTrue(statuses[-1]["executable"])
        self.assertTrue(statuses[-1]["published"])
        self.assertEqual(statuses[-1]["base_args"][1], "NO_MOTION")


if __name__ == "__main__":
    unittest.main()