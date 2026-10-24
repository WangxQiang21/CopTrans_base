"""The HUD must not mistake geometric availability for executable success."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.planning_display import PlanningDisplayState


class PlanningDisplayContracts(unittest.TestCase):
    def test_geometry_only_blocks_same_plan_playback_and_explains_state(self):
        state = PlanningDisplayState()
        self.assertTrue(state.accept_report({"plan_id": 1, "status": "pending"}))
        self.assertTrue(state.accept_report({"plan_id": 1, "status": "GEOMETRY_ONLY"}))
        self.assertFalse(state.accept_trajectory(1))
        self.assertIn("GEOMETRY ONLY", state.text())
        self.assertIn("EXECUTION NOT CERTIFIED", state.text())
        self.assertFalse(state.accept_report({"plan_id": 1, "status": "pending"}))
        self.assertTrue(state.accept_report({"plan_id": 2, "status": "pending"}))
        self.assertTrue(state.accept_trajectory(2))
        self.assertTrue(state.accept_report({"plan_id": 2, "status": "FEASIBLE"}))
        self.assertEqual(state.text(), "")
        self.assertFalse(state.accept_report({"plan_id": 1, "status": "GEOMETRY_ONLY"}))

    def test_malformed_or_stale_status_cannot_relabel_a_newer_trajectory(self):
        state = PlanningDisplayState()
        for report in (None, [], {"plan_id": True, "status": "FEASIBLE"},
                       {"plan_id": 1, "status": {}}, {"plan_id": 1, "status": "invented"}):
            self.assertFalse(state.accept_report(report))
        self.assertTrue(state.accept_trajectory(4))
        self.assertFalse(state.accept_trajectory(3))
        self.assertTrue(state.accept_report({"plan_id": 4, "status": "REJECTED"}))
        self.assertIn("NO CERTIFIED PATH", state.text())
        self.assertFalse(state.accept_trajectory(4))


if __name__ == "__main__":
    unittest.main()
