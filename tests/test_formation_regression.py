"""S10 fixed-case manifest and regression-runner tests (no ROS)."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import run_formation_regression as regression


CASES = ROOT / "tools/formation_eval_cases.yaml"


class ManifestTest(unittest.TestCase):

    def test_manifest_has_frozen_twenty_cases(self):
        manifest = regression.load_cases(CASES)
        self.assertEqual(
            [case["id"] for case in manifest["cases"]],
            list(regression.EXPECTED_IDS))
        self.assertEqual(len(manifest["sha256"]), 64)
        kinds = {case["kind"] for case in manifest["cases"]}
        self.assertEqual(
            kinds,
            {"pipeline", "evaluator", "request_invalid", "node", "group"})
        expectations = {case["expectation"] for case in manifest["cases"]}
        self.assertEqual(
            expectations,
            {"success", "must_fail", "scenario_pass", "statistical"})

    def test_group_cases_have_labeled_runs(self):
        manifest = regression.load_cases(CASES)
        for case in manifest["cases"]:
            if case["kind"] != "group":
                continue
            labels = [run["label"] for run in case["group_runs"]]
            self.assertGreaterEqual(len(labels), 3)
            self.assertEqual(len(labels), len(set(labels)))
        e19 = next(case for case in manifest["cases"] if case["id"] == "E19")
        self.assertEqual(
            [run["label"] for run in e19["group_runs"]],
            ["N1", "N2", "N3"])
        e20 = next(case for case in manifest["cases"] if case["id"] == "E20")
        self.assertEqual(len(e20["group_runs"]), 4)

    def test_manifest_rejects_duplicate_and_missing_ids(self):
        raw = yaml.safe_load(CASES.read_text(encoding="utf-8"))
        duplicate = copy.deepcopy(raw)
        duplicate["cases"][1]["id"] = "E01"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.yaml"
            path.write_text(yaml.safe_dump(duplicate), encoding="utf-8")
            with self.assertRaises(ValueError):
                regression.load_cases(path)
        malformed = copy.deepcopy(raw)
        malformed["schema"] = "wrong"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "schema.yaml"
            path.write_text(yaml.safe_dump(malformed), encoding="utf-8")
            with self.assertRaises(ValueError):
                regression.load_cases(path)


class GeometryAndRequestTest(unittest.TestCase):

    def test_grid_rasterises_obstacle_cells_and_edges(self):
        grid, info = regression.build_grid({
            "width": 40, "height": 30, "resolution": 0.1,
            "origin": [0.0, 0.0],
            "obstacles": [[1.0, 1.0, 1.5, 1.5]]})
        self.assertEqual(info["occupied_cells"], 25)
        self.assertAlmostEqual(info["free_fraction"], 1.0 - 25.0 / 1200.0)
        self.assertTrue(grid.occ[12, 12])
        self.assertFalse(grid.occ[5, 5])

    def test_request_mutation_hits_start_then_goal_field(self):
        manifest = regression.load_cases(CASES)
        by_id = {case["id"]: case for case in manifest["cases"]}
        for case_id, expected in (("E11", "start_center"),
                                  ("E12", "goal_center")):
            request = regression.mutate_request(
                regression.request_message(by_id[case_id], 101),
                by_id[case_id]["request_mutation"])
            validation = regression.validate_formation_request(request)
            self.assertFalse(validation.valid)
            self.assertEqual(validation.reason, "INVALID_REQUEST")
            self.assertIn(expected, validation.field)

    def test_plan_ids_are_stable_and_valid_uint32(self):
        first = regression.plan_id_for("E01")
        self.assertEqual(first, regression.plan_id_for("E01"))
        self.assertNotEqual(first, regression.plan_id_for("E02"))
        self.assertGreaterEqual(first, 1)
        self.assertLessEqual(first, 2000000000)

    def test_nan_and_empty_request_messages_fail_closed(self):
        manifest = regression.load_cases(CASES)
        e18 = next(case for case in manifest["cases"] if case["id"] == "E18")
        request = regression.request_message(e18, 201)
        request.vehicles[0].start_center.x = float("nan")
        validation = regression.validate_formation_request(request)
        self.assertFalse(validation.valid)
        empty = regression.request_message(e18, 202)
        empty.vehicles = []
        validation = regression.validate_formation_request(empty)
        self.assertFalse(validation.valid)
        self.assertEqual(validation.field, "vehicles")


class TamperAndOutcomeTest(unittest.TestCase):

    def _load(self, case_id):
        manifest = regression.load_cases(CASES)
        return next(case for case in manifest["cases"]
                    if case["id"] == case_id)

    def test_mode_control_tamper_reaches_independent_evaluator(self):
        case = self._load("E13")
        run = regression.run_once(case, {
            "allowed_modes": None, "enable_nlp": False, "record_only": False})
        self.assertTrue(run["harness_ok"], run["harness_error"])
        self.assertFalse(run["feasible"])
        self.assertEqual(run["reason"], "KINEMATIC_INFEASIBLE")
        self.assertEqual(run["tamper"]["field"], "mode_control")
        self.assertGreater(run["tamper"]["point_index"], 0)

    def test_shared_time_tamper_fails_message_contract(self):
        case = self._load("E16")
        run = regression.run_once(case, {
            "allowed_modes": None, "enable_nlp": False, "record_only": False})
        self.assertTrue(run["harness_ok"], run["harness_error"])
        self.assertEqual(run["status"], "INVALID")
        self.assertEqual(run["reason"], "INVALID_TIMED_TRAJECTORY")
        self.assertEqual(run["failure_field"], "time_from_start")

    def test_failed_expectation_is_retained_not_dropped(self):
        manifest = regression.load_cases(CASES)
        case = copy.deepcopy(next(item for item in manifest["cases"]
                                  if item["id"] == "E01"))
        case["expectation"] = "must_fail"
        case["expected_failure_codes"] = ["KINEMATIC_INFEASIBLE"]
        options = {"allowed_modes": None, "enable_nlp": False,
                   "record_only": False}
        run = regression.run_once(case, options)
        row = regression.aggregate_case(case, [run], options)
        self.assertTrue(run["feasible"])
        self.assertFalse(run["expectation_met"])
        self.assertFalse(row["passed"])
        self.assertEqual(row["outcome_rate"]["numerator"], 0)
        self.assertEqual(row["outcome_rate"]["denominator"], 1)
        self.assertEqual(row["repeat_count"], 1)

    def test_ablation_is_record_only_and_uses_allowed_modes(self):
        case = self._load("E03")
        options = {"allowed_modes": (0, 1), "enable_nlp": False,
                   "record_only": True, "ablation": "no_spin"}
        run = regression.run_once(case, options)
        self.assertTrue(run["harness_ok"], run["harness_error"])
        self.assertEqual(run["expectation_mode"], "record_only")
        self.assertTrue(run["expectation_met"])
        self.assertFalse(run["feasible"])
        self.assertTrue(run["failure_codes"])

    def test_node_scenario_clears_nan_empty_and_stale_plan(self):
        case = self._load("E18")
        run = regression.run_once(case, {
            "allowed_modes": None, "enable_nlp": False, "record_only": False})
        self.assertTrue(run["harness_ok"], run["harness_error"])
        self.assertTrue(run["scenario_ok"])
        self.assertEqual(run["status"], "SCENARIO_CLEARED")
        steps = {step["step"]: step for step in run["scenario_steps"]}
        self.assertTrue(steps["publish"]["trajectory_published"])
        for name in ("invalid_request:nan_start",
                     "invalid_request:empty_vehicles"):
            self.assertTrue(steps[name]["clear"])
            self.assertFalse(steps[name]["trajectory_published"])
        self.assertFalse(
            steps["stale_seed_after_invalid"]["trajectory_published"])


class RunnerIntegrationTest(unittest.TestCase):

    def test_summary_schema_and_three_repeat_aggregation(self):
        with tempfile.TemporaryDirectory() as directory:
            summary, path = regression.run_regression(
                CASES, directory, repeats=2, ids=["E11", "E13", "E18"])
            self.assertTrue(path.exists())
            data = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(data["schema"], "formation_regression/v1")
            self.assertEqual(data["config"]["repeats"], 2)
            self.assertTrue(data["all_pass"])
            self.assertEqual(data["case_count"], 3)
            self.assertEqual(data["passed_case_count"], 3)
            self.assertEqual(data["failures"], [])
            for row in data["rows"]:
                self.assertEqual(row["repeat_count"], 2)
                self.assertTrue(row["passed"])
                self.assertEqual(len(row["runs"]), 2)
                self.assertIsNotNone(row["wall_s"]["median"])
            self.assertEqual(summary["schema"], data["schema"])

    def test_group_case_aggregates_subrun_success(self):
        with tempfile.TemporaryDirectory() as directory:
            summary, _ = regression.run_regression(
                CASES, directory, repeats=1, ids=["E19"])
            row = summary["rows"][0]
            self.assertEqual(row["kind"], "group")
            self.assertTrue(row["passed"])
            self.assertEqual(row["outcome_rate"]["denominator"], 3)
            self.assertEqual(row["outcome_rate"]["numerator"], 3)
            labels = [sub["label"]
                      for sub in row["group_subruns"][0]]
            self.assertEqual(labels, ["N1", "N2", "N3"])

    def test_cli_requires_three_repeats(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(SystemExit):
                regression.main([
                    "--output-dir", directory, "--repeats", "2"])


if __name__ == "__main__":
    unittest.main()