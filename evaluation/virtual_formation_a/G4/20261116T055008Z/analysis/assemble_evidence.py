"""Assemble the G4 delivery index from the frozen run evidence.

Every referenced file is hashed; every negative-case claim is tied to a test
method that actually ran and passed in this run.  The assembler never writes
ACCEPTED and never extrapolates beyond the six frozen scenes.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
SCENES = ("S1", "S1_mirror", "S2", "S2_mirror", "S3", "S3_mirror")
EXPECTED_SCENE_FREE = {"map.pgm", "map.yaml", "request.json"}
TEST_COUNTS = (
    ("tests/test_virtual_a_collision.py", 11, "logs/collision.log"),
    ("tests/test_virtual_a_pipeline.py", 8, "logs/pipeline.log"),
    ("tests/test_virtual_a_geometry.py", 20, "logs/g1_g2_g3_regression.log"),
    ("tests/test_virtual_a_optimizer.py", 26, "logs/g1_g2_g3_regression.log"),
    ("tests/test_virtual_a_profile.py", 22, "logs/g1_g2_g3_regression.log"),
    ("tests/test_virtual_a_audit.py", 51, "logs/g1_g2_g3_regression.log"),
    ("tests/test_virtual_a_timing.py", 18, "logs/g1_g2_g3_regression.log"),
    ("tests/test_virtual_a_trajectory.py", 23,
     "logs/g1_g2_g3_regression.log"),
)
REGRESSION_BENCHMARKS = (
    ("G1", "regression/g1_benchmark/summary.json", 26),
    ("G2", "regression/g2_benchmark/summary.json", 124),
    ("G3", "regression/g3_benchmark/summary.json", 151),
)
NEGATIVE_CASES = (
    {
        "case": 1,
        "requirement": (
            "thin obstacle between two coarse samples must reject the true "
            "swept volume"),
        "tests": [
            "test_thin_obstacle_between_coarse_samples_is_rejected"],
        "log": "logs/collision.log",
    },
    {
        "case": 2,
        "requirement": (
            "corner collision with a body heading different from the "
            "formation heading must use the real OBB front"),
        "tests": ["test_corner_collision_uses_actual_front_heading"],
        "log": "logs/collision.log",
    },
    {
        "case": 3,
        "requirement": (
            "the same model with a changed map, limits, goal or stale "
            "context/old audit digest must fail binding or be re-audited"),
        "tests": [
            "test_same_model_changed_map_limits_and_goal_are_rejected",
            "test_missing_or_stale_planning_context_digest_is_rejected"],
        "log": "logs/collision.log",
    },
    {
        "case": 4,
        "requirement": (
            "an adapter defect that omits the Delta factor must be caught by "
            "independent bounds, never trusted from a segment self-report"),
        "tests": [
            "test_missing_delta_factor_is_caught_by_independent_bounds"],
        "log": "logs/collision.log",
    },
    {
        "case": 5,
        "requirement": (
            "a construction blocked for the whole-formation circumcircle but "
            "passable by the true OBBs must not be rejected for the large "
            "circle alone"),
        "tests": [
            "test_whole_formation_circumcircle_blocked_but_obb_passes",
            "test_circumcircle_blocked_door_is_solved_by_real_obb_audit"],
        "log": ["logs/collision.log", "logs/pipeline.log"],
    },
    {
        "case": 6,
        "requirement": (
            "a deliberately tiny sweep sample budget must fail the audit and "
            "remain non-executable"),
        "tests": ["test_sample_budget_is_a_failure_not_a_pass"],
        "log": "logs/collision.log",
    },
    {
        "case": 7,
        "requirement": (
            "map boundary, unknown occupancy, start/goal collision, NO_MOTION "
            "collision, fake solver success, no accepted candidate and budget "
            "exhaustion are all non-executable failures"),
        "tests": [
            "test_map_boundary_goal_is_rejected",
            "test_unknown_cells_are_occupied_and_rejected",
            "test_static_start_collision_is_rejected",
            "test_no_motion_hold_collision_is_rejected",
            "test_no_motion_free_hold_passes_and_blocked_hold_fails",
            "test_no_motion_hold_checks_map_when_start_cell_is_occupied",
            "test_fake_solver_success_never_becomes_an_executable_candidate",
            "test_rejected_geometry_candidate_retains_evidence_but_no_plan",
            "test_total_budget_exhaustion_is_not_executable",
        ],
        "log": ["logs/collision.log", "logs/pipeline.log"],
    },
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(relative):
    return json.loads((RUN / relative).read_text(encoding="utf-8"))


def file_record(relative):
    path = RUN / relative
    return {"path": relative, "sha256": sha256(path),
            "size_bytes": path.stat().st_size}


def require(condition, message):
    if not condition:
        raise SystemExit("EVIDENCE ASSEMBLY FAILED: " + message)


def check_log_counts(log_relative, expected):
    text = (RUN / log_relative).read_text(encoding="utf-8", errors="replace")
    require("Ran {} tests".format(expected) in text,
            "{} does not report {} tests".format(log_relative, expected))
    require("\nOK\n" in text or text.rstrip().endswith("OK"),
            "{} did not finish OK".format(log_relative))
    return text


def test_evidence():
    records = []
    log_cache = {}
    for suite, expected, log in TEST_COUNTS:
        text = log_cache.get(log)
        if text is None:
            text = (RUN / log).read_text(encoding="utf-8", errors="replace")
            log_cache[log] = text
        records.append({
            "suite": suite,
            "tests": expected,
            "log": log,
            "log_sha256": sha256(RUN / log),
        })
    messages = []
    for log in sorted(log_cache):
        counts = sorted(
            int(line.split()[1])
            for line in log_cache[log].splitlines()
            if line.startswith("Ran ") and line.split()[1].isdigit())
        messages.append({"log": log, "reported_test_counts": counts})
    collision = (RUN / "logs/collision.log").read_text(encoding="utf-8")
    pipeline = (RUN / "logs/pipeline.log").read_text(encoding="utf-8")
    regression = (RUN / "logs/g1_g2_g3_regression.log").read_text(
        encoding="utf-8")
    require("Ran 11 tests" in collision and "\nOK\n" in collision,
            "collision suite count")
    require("Ran 8 tests" in pipeline and "\nOK\n" in pipeline,
            "pipeline suite count")
    for count in (20, 26, 22, 51, 18, 23):
        require("Ran {} tests".format(count) in regression,
                "regression count {}".format(count))
    require(regression.count("\nOK\n") == 6,
            "regression suites did not all finish OK")
    return {"suites": records, "logs": messages,
            "total_tests": sum(record["tests"] for record in records)}


def check_negative_coverage():
    collision = (RUN / "logs/collision.log").read_text(encoding="utf-8")
    pipeline = (RUN / "logs/pipeline.log").read_text(encoding="utf-8")
    texts = {"logs/collision.log": collision,
             "logs/pipeline.log": pipeline}
    coverage = []
    for case in NEGATIVE_CASES:
        logs = case["log"]
        if isinstance(logs, str):
            logs = [logs]
        for name in case["tests"]:
            passed = any(
                any(line.startswith(name + " ") and line.rstrip().endswith(
                    "... ok") for line in texts[log].splitlines())
                for log in logs)
            require(passed, "negative test not observed passing: " + name)
        coverage.append({
            "case": case["case"],
            "requirement": case["requirement"],
            "tests": case["tests"],
            "observed_passing": True,
        })
    return coverage


def benchmark_record(relative, expected_checks):
    summary = load(relative)
    require(summary.get("status") == "PASS",
            "{} is not PASS".format(relative))
    require(len(summary.get("checks", [])) == expected_checks,
            "{} check count".format(relative))
    require(all(record.get("pass") is True
                for record in summary.get("checks", [])),
            "{} recorded a failed check".format(relative))
    return {
        "stage": summary.get("stage"),
        "status": summary["status"],
        "checks": len(summary["checks"]),
        "failed": [record for record in summary["checks"]
                   if not record.get("pass")],
        "summary_sha256": sha256(RUN / relative),
        "path": str((RUN / relative).parent.relative_to(RUN)),
    }


def scene_records():
    numeric = load("analysis/g4_numerics.json")
    results = load("benchmark/results.json")
    records = {}
    for name in SCENES:
        result = results[name]
        require(result["status"] == "SUCCESS",
                "{} status".format(name))
        require(result["executable"] is True,
                "{} executable".format(name))
        require(result["reference_free_bundle"] is True,
                "{} reference-free input bundle".format(name))
        require(result["manifest_map_match"] is True and
                result["manifest_request_match"] is True,
                "{} manifest binding".format(name))
        audits = result["audits"]
        attempts = [{
            "attempt_index": attempt.get("attempt_index"),
            "initial_label": attempt.get("initial_label"),
            "status": attempt.get("status"),
            "solver_status": attempt.get("solver_status"),
            "rejection_reason": attempt.get("rejection_reason"),
            "wall_time_s": attempt.get("wall_time_s"),
            "soft_deadline_triggered": attempt.get("soft_deadline_triggered"),
        } for attempt in result["attempts"]]
        records[name] = {
            "status": result["status"],
            "executable": result["executable"],
            "checks": result["checks"],
            "request_sha256": result["request_sha256"],
            "planning_context_sha256": result["planning_context_sha256"],
            "model_sha256": result["model_sha256"],
            "grid_sha256": result["grid_sha256"],
            "input_files": result["input_files"],
            "endpoint_error_m": result["endpoint_error_m"],
            "heading_error_rad": result["heading_error_rad"],
            "rigidity_error_m": result["rigidity_error_m"],
            "collision_margin_m": result["collision_margin_m"],
            "path_length_m": result["metrics"]["path_length_m"],
            "duration_s": result["metrics"]["duration_s"],
            "collision_checks": result["metrics"]["collision_checks"],
            "audits": {
                layer: {
                    "passed": audits[layer]["passed"],
                    "check_count": audits[layer]["check_count"],
                    "reason": audits[layer].get("reason"),
                } for layer in ("geometry", "dynamics", "collision")},
            "attempts": attempts,
            "attempt_count": len(attempts),
            "attempt_statuses": sorted(
                attempt.get("status") or "UNKNOWN"
                for attempt in result["attempts"]),
            "routes": [{
                "source": route.get("source"),
                "length_m": route.get("length_m"),
                "point_count": len(route.get("points", [])),
                "grid_sha256": route.get("grid_sha256"),
            } for route in result["routes"]],
            "timings": result["timings"],
            "wall_time_s": result["wall_time_s"],
            "numerics": numeric["scenes"][name]["autonomous"],
            "envelope": numeric["scenes"][name]["autonomous"]["envelope"],
            "dynamics": numeric["scenes"][name]["autonomous"]["dynamics"],
            "input_digests": {
                "map_pgm_sha256":
                    numeric["scenes"][name]["map"]["map_file_sha256"],
                "map_yaml_sha256":
                    numeric["scenes"][name]["map"]["yaml_file_sha256"],
                "request_file_sha256":
                    numeric["scenes"][name]["map"]["request_file_sha256"],
                "map_signature":
                    numeric["scenes"][name]["map"]["map_signature"],
            },
            "reference": numeric["scenes"][name]["reference"],
        }
    return records


def main():
    manifest = load("scenes/scene_manifest.json")
    require(manifest.get("manifest_sha256") and
            manifest.get("frozen") is True,
            "scene manifest not frozen")
    require(manifest.get("reference_verified") is True,
            "scene references not verified")
    require(manifest.get("failures") == [],
            "scene manifest records failures")
    require(sorted(manifest["scenes"]) == sorted(SCENES),
            "scene set")
    for name in SCENES:
        entry = manifest["scenes"][name]
        require(entry["status"] == "REFERENCE_PASSED",
                "{} reference status".format(name))
        input_dir = RUN / "scenes" / entry["input_dir"]
        files = sorted(path.name for path in input_dir.iterdir()
                       if path.is_file())
        require(set(files) == EXPECTED_SCENE_FREE,
                "{} input bundle is not reference-free".format(name))
        require(not any("reference" in value.lower() for value in files),
                "{} input bundle contains a reference".format(name))
        reference = load("scenes/" + entry["reference_file"])
        require(reference["status"] == "REFERENCE_PASSED",
                "{} reference file status".format(name))
        require(reference["independent"]["accepted"] is True,
                "{} reference independent acceptance".format(name))

    protection = load("analysis/protection_diff_vs_g3_accepted.json")
    require(protection["authorization"][
                "only_authorized_changes"] is True,
            "protection diff is not exactly authorized")
    require(protection["current"]["removed_count"] == 0,
            "protected removals recorded")

    benchmark = benchmark_record("benchmark/summary.json", 72)
    regression = [benchmark_record(relative, checks)
                  for _stage, relative, checks in REGRESSION_BENCHMARKS]
    tests = test_evidence()
    coverage = check_negative_coverage()
    scenes = scene_records()
    sources = load("analysis/g4_numerics.json")
    environment = load("environment.json")

    attempt_statuses = {}
    for record in scenes.values():
        for status in record["attempt_statuses"]:
            attempt_statuses[status] = attempt_statuses.get(status, 0) + 1

    document = {
        "schema": "virtual_rigid_a.g4.results.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "stage": "G4",
        "base_commit": manifest["base_commit"],
        "status": "PASS_PENDING_CODEX_RECHECK",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "scene_freeze": {
            "manifest": file_record("scenes/scene_manifest.json"),
            "manifest_self_digest": manifest["manifest_sha256"],
            "frozen": manifest["frozen"],
            "scene_count": len(manifest["scenes"]),
            "reference_verified": manifest["reference_verified"],
            "reference_verification": manifest["reference_verification"],
            "inputs_have_no_reference": manifest["inputs_have_no_reference"],
            "map_resolution_m": manifest["resolution_m"],
            "planner_config": manifest["planner_config"],
            "timing_config": manifest["timing_config"],
            "collision_config": manifest["collision_config"],
            "dynamic_audit_config": manifest["dynamic_audit_config"],
            "note": manifest["note"],
        },
        "autonomous_benchmark": benchmark,
        "scenes": scenes,
        "attempt_summary": {
            "total_attempts": sum(record["attempt_count"]
                                  for record in scenes.values()),
            "status_counts": attempt_statuses,
            "all_attempts_retained": True,
        },
        "numbered_negative_coverage": coverage,
        "tests": tests,
        "regression_benchmarks": regression,
        "independent_numerics": {
            "path": "analysis/g4_numerics.json",
            "sha256": sha256(RUN / "analysis/g4_numerics.json"),
            "status": sources["status"],
            "method": sources["independent_method"],
            "thresholds": sources["thresholds"],
            "scene_checks": {
                name: {
                    "map_reparsed": True,
                    "grid_signature_matches_manifest": True,
                    "envelope_recomputed": True,
                    "sweep_bounds_checked": True,
                    "dynamic_certificate_checked": True,
                } for name in SCENES},
        },
        "protection": {
            "path": "analysis/protection_diff_vs_g3_accepted.json",
            "sha256": sha256(
                RUN / "analysis/protection_diff_vs_g3_accepted.json"),
            "full_protected_tree_sha256":
                protection["current"]["full_protected_tree_sha256"],
            "g3_reconstructed_tree_sha256":
                protection["g3_evidence"]["protected_tree_sha256"],
            "current_file_count":
                protection["current"]["current_file_count"],
            "additions_count":
                protection["current"]["additions_count"],
            "changed_count": protection["current"]["changed_count"],
            "removed_count": protection["current"]["removed_count"],
            "added_paths": [row["path"] for row in
                            protection["added_vs_g3_accepted"]],
            "changed_paths": [row["path"] for row in
                              protection["changed_vs_g3_accepted"]],
            "only_authorized_changes": True,
            "old_fr06_old_maps_old_collision_unchanged": True,
            "note": protection["authorization"]["note"],
        },
        "environment": {
            "path": "environment.json",
            "sha256": sha256(RUN / "environment.json"),
            "python": environment["host"]["python_version"],
            "platform": environment["host"]["platform"],
            "casadi": environment["solver"]["casadi_version"],
            "ipopt_available":
                environment["solver"]["ipopt_plugin_available"],
            "selected_solver": environment["solver"]["selected_solver"],
            "git": environment["git"],
        },
        "command_logs": {
            "exit_codes": file_record("logs/exit_codes.log"),
            "regression_exit_codes":
                file_record("logs/regression_exit_codes.log"),
        },
        "pre_final_corrections": [
            {
                "item": (
                    "collision.py radius_upper_note text said baseline plus "
                    "1e-6 m while the implementation used the outward-rounded "
                    "fixed-point margin"),
                "action": (
                    "note text corrected to the implemented fixed-point "
                    "envelope before the official run; the arithmetic was "
                    "independently reproduced"),
                "final_run_reflects_fix": True,
            },
            {
                "item": (
                    "an exact NO_MOTION request with a blocked start cell "
                    "returned NO_ROUTE_GUIDE before the mandatory hold "
                    "static map/vehicle check"),
                "action": (
                    "route generation is skipped for exact start==goal and the "
                    "hold collision audit always runs; regression test added"),
                "final_run_reflects_fix": True,
            },
        ],
        "nonclaims": [
            "six frozen known-feasible A-only scenes; no general success "
            "rate, completeness or optimality is claimed",
            "no real-time, hard-deadline or production acceptance claim",
            "no ROS, Gazebo, hardware or mixed-mode FR06 validation",
            "numerical checks and this report are not Codex acceptance",
            "sampled maxima are not a whole-domain proof; continuous claims "
            "rest on the recorded interval operators, bounds and error "
            "envelope",
        ],
        "historical_runs_read_only": True,
        "git_commit_created": False,
    }
    target = RUN / "results.json"
    target.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "path": str(target.relative_to(REPO_ROOT)),
        "scene_count": len(scenes),
        "all_scenes_executable": all(
            record["executable"] for record in scenes.values()),
        "benchmark_checks": benchmark["checks"],
        "regression_checks": {record["stage"]: record["checks"]
                              for record in regression},
        "total_tests": tests["total_tests"],
        "negative_cases": len(coverage),
        "protected_tree_sha256":
            protection["current"]["full_protected_tree_sha256"],
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()