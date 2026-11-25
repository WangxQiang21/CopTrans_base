"""Check actual ROS serialization and archive identity across historical map changes."""
import gzip
import json
from pathlib import Path
import unittest

from cases import ROOT
from historical_routes import historical_fixture, replay_request, ros_contract


RUNS = ROOT / "reports/runs"
ORIGINAL = "20261125_115519_smXjHv"
WEST = "20261125_120655_6xtU9x"
AUTHORITY = "20261125_173104_e49921"


@unittest.skipUnless((RUNS / ORIGINAL).is_dir(), "local historical run archive required")
class HistoricalRoutes(unittest.TestCase):
    def test_selected_endpoints_and_revisions(self):
        cfg, _, old = historical_fixture(ORIGINAL, "formation", "standard")
        cfg_west, _, west = historical_fixture(WEST, "formation", "standard")
        self.assertEqual(cfg["start"], cfg_west["start"])
        self.assertEqual(cfg["goal"], cfg_west["goal"])
        self.assertNotEqual(old["map_sha256"],west["map_sha256"])
        self.assertTrue(old["source_spawn_matches"] and old["old_timing_matches"])

    @unittest.skipUnless((RUNS / AUTHORITY / "trajectories/model.json.gz").exists(), "local authoritative ROS record required")
    def test_ros_authority_field_equality_and_fitted_start(self):
        selected = json.loads((RUNS / ORIGINAL / "experiment.json").read_bytes())["start"]
        payload = replay_request(RUNS / ORIGINAL,"formation","standard")
        with gzip.open(RUNS / AUTHORITY / "trajectories/model.json.gz") as stream:
            authority = json.load(stream)["request"]
        self.assertNotEqual(selected[0], payload["request"]["start"]["reference"][0])
        self.assertEqual(ros_contract(payload["request"],"cargo_wedge"), authority)

    def test_explicit_goal_single_pose_stamped(self):
        native = replay_request(RUNS / ORIGINAL, "single", "baseline", goal=[15., 17., .64])["request"]
        self.assertEqual(native["goal"]["reference"], [15., 17., .6399999999999999])
        self.assertEqual(native["goal"]["heading"], [.6399999999999999, 0., 0.])
        self.assertEqual(native["limits"]["min_motion_duration"], .3)


if __name__ == "__main__":
    unittest.main()
