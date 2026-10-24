"""Durability contracts for generated planner reports."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.diagnostics import atomic_write_json


class AtomicReportContracts(unittest.TestCase):
    def test_report_replacement_is_complete_and_leaves_no_temporary_file(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "plan.json"
            atomic_write_json(target, {"version": 1, "items": [1, 2]})
            atomic_write_json(target, {"version": 2, "items": [3]})
            self.assertEqual(json.loads(target.read_text()),
                             {"version": 2, "items": [3]})
            self.assertEqual([item.name for item in Path(directory).iterdir()],
                             ["plan.json"])

    def test_non_finite_report_does_not_replace_the_previous_file(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "plan.json"
            atomic_write_json(target, {"status": "FEASIBLE"})
            with self.assertRaises(ValueError):
                atomic_write_json(target, {"speed": float("nan")})
            self.assertEqual(json.loads(target.read_text()),
                             {"status": "FEASIBLE"})
            self.assertEqual([item.name for item in Path(directory).iterdir()],
                             ["plan.json"])


if __name__ == "__main__":
    unittest.main()
