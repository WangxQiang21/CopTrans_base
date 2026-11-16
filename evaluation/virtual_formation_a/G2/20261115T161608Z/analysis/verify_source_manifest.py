"""Independently verify the G2 source-and-evidence manifest.

Run with ``python3 -B -I``.  Only the standard library is used.  Every source
and run-file hash, both protected snapshots, the G1 acceptance binding, the
frozen config/fixtures, the acceptance maxima and the git state are recomputed
from disk.  The verifier writes nothing except its console log (captured by
tee outside this process).
"""
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
MANIFEST_PATH = RUN / "source_manifest.json"
MANIFEST = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
SCHEMA = "virtual_rigid_a.g2.source_manifest.v1"
STAGE = "G2"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
G1_ACCEPTANCE = (REPO_ROOT / "evaluation/virtual_formation_a/G1" /
                 "codex_acceptance_rework_01/CODEX_ACCEPTANCE.json")
G1_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G1/20261115T152858Z"
G1_MANIFEST = G1_RUN / "source_manifest.json"
G1_PROTECTED = G1_RUN / "legacy/protected_after.json"
G0_SNAPSHOT = (REPO_ROOT / "evaluation/virtual_formation_a/G0" /
               "20261115T143321Z_rework/protected_before.json")
EXPECTED_G1_ACCEPTANCE_SHA = (
    "6a1442cbdf2648b03c3add8dd2ba5ee6bd061d3deba9539dbd72089a6b1b2230")
EXPECTED_G1_MANIFEST_SHA = (
    "fa6ca4387bf57d35f76097f5e50c23715bf72256f186707b59f4e1ec4262718a")
EXPECTED_G1_TREE = (
    "910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519")
EXPECTED_G1_AFTER_SHA = (
    "c9e4c874c3e7d0dc05a1850d74febd28ec7475a232b6f8f98d3e1c11d2892470")
EXPECTED_G0_SHA = (
    "a89efe0887eb20aced4bad74bc8adf280079191b9c491436e2ae4e89c2506c9a")
EXPECTED_G0_TREE = (
    "ff58f2b5c6acfbc207327202f875e5e9a20734ef09ebb6070ee07edce6ebf091")
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
G2_SOURCES = {
    PACKAGE + "audit.py",
    PACKAGE + "optimizer.py",
    "docs/virtual_formation_a/CLAUDE_G2_TASK.md",
    "tests/fixtures/virtual_a/existence_reference.json",
    "tests/fixtures/virtual_a/open_requests.json",
    "tests/test_virtual_a_audit.py",
    "tests/test_virtual_a_optimizer.py",
}
EXPECTED_CHANGED = {
    PACKAGE + "contracts.py",
    "tools/virtual_a_benchmark.py",
}
REFERENCE_DOCUMENTS = {
    "docs/VIRTUAL_FORMATION_A_FR06_TECHNICAL_REVIEW_20261115.md",
}
EXCLUSIONS = {
    "source_manifest.json",
    "logs/source_manifest_generation.log",
    "logs/source_manifest_verification.log",
}
EXPECTED_TEST_COUNTS = {
    "audit": 20, "optimizer": 12, "g1_geometry": 20, "g1_profile": 22}
EXPECTED_CHECK_COUNT = 97
EXPECTED_RUN_FILE_COUNT = 30
EXPECTED_NEGATIVE_COVERAGE_COUNT = 24
REQUIRED_NEGATIVE_TESTS = {
    "test_non_c2_profile_rejected",
    "test_car1_wrong_initial_heading_rejected",
    "test_forged_request_digest_rejected",
    "test_forged_goal_is_checked_against_original_request",
    "test_wrong_vehicle_shape_rejected",
    "test_invalid_limit_rejected",
    "test_nan_goal_rejected_before_any_geometry_claim",
    "test_static_wheel_limit_uses_whole_segment_envelope",
    "test_segment_interior_curvature_violation_with_legal_nodes",
    "test_unconfirmed_enclosure_is_not_published_as_feasible",
    "test_solver_exception_maps_to_numerical_failure",
    "test_small_budget_returns_budget_exhausted_without_path",
    "test_fake_solver_success_wrong_endpoint_is_not_feasible",
    "test_fake_solver_success_curvature_violation_is_not_feasible",
    "test_exact_hold_is_no_motion_but_near_or_turned_goal_is_not",
}
NAMES = ("E1", "E2", "E3", "E4", "E5", "E6",
         "STRAIGHT_LINE", "STRAIGHT_POLICY")
FAILURES = []


def check(condition, message):
    if not condition:
        FAILURES.append(message)
    return bool(condition)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def canonical_tree(entries):
    payload = {
        entry["path"]: {
            "type": entry["type"],
            "sha256": entry["sha256"],
            "size_bytes": entry["size_bytes"],
        }
        for entry in entries
    }
    return hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")).hexdigest()


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(REPO_ROOT), check=True,
        capture_output=True, text=True).stdout


def verify_header():
    check(MANIFEST["schema"] == SCHEMA, "schema changed")
    check(MANIFEST["stage"] == STAGE, "stage changed")
    check(MANIFEST["run_id"] == RUN.name, "run id changed")
    check(MANIFEST["baseline_head"] == BASELINE_HEAD, "baseline changed")
    check(MANIFEST["repo_root"] == str(REPO_ROOT), "repo root changed")
    check(sorted(MANIFEST["excluded_self_referential"]) ==
          sorted(EXCLUSIONS), "exclusion set changed")
    check(MANIFEST["source_file_count"] == 22, "source count changed")
    check(MANIFEST["source_file_count"] == len(MANIFEST["source_files"]),
          "source count field is inconsistent")
    check(MANIFEST["run_file_count"] == EXPECTED_RUN_FILE_COUNT,
          "run-file count changed")
    check(MANIFEST["reference_document_count"] == 1,
          "reference document count changed")
    check(MANIFEST["reference_document_count"] ==
          len(MANIFEST["reference_documents"]),
          "reference document count field is inconsistent")
    check(MANIFEST["verification"]["script"] ==
          "analysis/verify_source_manifest.py",
          "manifest verification script changed")
    check(MANIFEST["results"]["attempt_count"] == 48,
          "manifest attempt aggregate changed")
    check(MANIFEST["results"]["benchmark_checks"] == EXPECTED_CHECK_COUNT,
          "manifest benchmark aggregate changed")
    check(MANIFEST["results"]["benchmark_status"] == "PASS",
          "manifest benchmark status changed")
    check(MANIFEST["results"]["all_audits_passed"] is True,
          "manifest audit aggregate changed")
    check(MANIFEST["results"]["g2_entered"] is True,
          "manifest G2 entry changed")
    check(MANIFEST["results"]["g3_entered"] is False,
          "manifest G3 entry changed")


def verify_sources():
    for relative, record in MANIFEST["source_files"].items():
        path = REPO_ROOT / relative
        if not check(path.is_file(), "missing source: " + relative):
            continue
        check(sha256(path) == record["sha256"],
              "source hash changed: " + relative)
        check(path.stat().st_size == record["bytes"],
              "source size changed: " + relative)
    check(set(MANIFEST["source_files"]) ==
          set(MANIFEST["changed_accepted_sources"]) |
          set(MANIFEST["g2_source_files"]),
          "source partition is inconsistent")
    check(set(MANIFEST["g2_source_files"]) == G2_SOURCES,
          "G2 source list changed")
    for relative in G2_SOURCES:
        check(relative in MANIFEST["source_files"],
              "G2 source not bound: " + relative)
    for relative, record in MANIFEST["reference_documents"].items():
        check(relative in REFERENCE_DOCUMENTS,
              "unexpected reference document: " + relative)
        check(sha256(REPO_ROOT / relative) == record["sha256"],
              "reference document hash changed: " + relative)
    changed = []
    for relative, record in MANIFEST["changed_accepted_sources"].items():
        current = sha256(REPO_ROOT / relative)
        check(current == record["current_sha256"],
              "accepted source current hash changed: " + relative)
        if record["changed"]:
            changed.append(relative)
            check(current == record["current_sha256"] and
                  current != record["g1_sha256"],
                  "changed source does not differ from G1: " + relative)
        else:
            check(current == record["g1_sha256"],
                  "unchanged accepted source differs: " + relative)
    check(set(changed) == EXPECTED_CHANGED,
          "authorized G1 source changes changed: {}".format(sorted(changed)))
    check(sorted(MANIFEST["changed_accepted_source_list"]) ==
          sorted(EXPECTED_CHANGED),
          "changed accepted-source list changed")


def verify_run_files():
    for relative, record in MANIFEST["run_files"].items():
        path = RUN / relative
        if not check(path.is_file(), "missing run file: " + relative):
            continue
        check(sha256(path) == record["sha256"],
              "run file hash changed: " + relative)
        check(path.stat().st_size == record["bytes"],
              "run file size changed: " + relative)
    check("analysis/verify_source_manifest.py" in MANIFEST["run_files"],
          "the verifier itself is not bound by the manifest")
    actual = set()
    for path in RUN.rglob("*"):
        if path.is_file():
            relative = path.relative_to(RUN).as_posix()
            if relative not in EXCLUSIONS:
                actual.add(relative)
    check(actual == set(MANIFEST["run_files"]),
          "run-file set mismatch: missing={} extra={}".format(
              sorted(set(MANIFEST["run_files"]) - actual)[:5],
              sorted(actual - set(MANIFEST["run_files"]))[:5]))
    check(MANIFEST["run_file_count"] == len(MANIFEST["run_files"]),
          "run-file count field changed")
    total = sum(record["bytes"] for record in MANIFEST["run_files"].values())
    check(MANIFEST["run_total_bytes"] == total, "run byte total changed")
    check(not [path for path in RUN.rglob("*")
               if path.name == "__pycache__"
               or path.suffix in (".pyc", ".pyo")],
          "generated bytecode exists under the run")


def verify_protected():
    protection = load(RUN / "analysis" /
                      "protection_diff_vs_g1_accepted.json")
    g0 = load(G0_SNAPSHOT)
    check(sha256(G0_SNAPSHOT) == EXPECTED_G0_SHA, "G0 snapshot changed")
    check(g0["protected_tree_sha256"] == EXPECTED_G0_TREE,
          "G0 declared tree changed")
    check(canonical_tree(g0["protected_files"]) == EXPECTED_G0_TREE,
          "G0 snapshot content changed")
    check(MANIFEST["protected"]["g0_sha256"] == EXPECTED_G0_SHA,
          "manifest G0 hash changed")
    check(MANIFEST["protected"]["g0_tree_sha256"] == EXPECTED_G0_TREE,
          "manifest G0 tree changed")

    acceptance = load(G1_ACCEPTANCE)
    check(sha256(G1_ACCEPTANCE) == EXPECTED_G1_ACCEPTANCE_SHA,
          "G1 acceptance file changed")
    check(acceptance["status"] == "ACCEPTED", "G1 is not ACCEPTED")
    check(acceptance["source_manifest_sha256"] ==
          EXPECTED_G1_MANIFEST_SHA, "G1 acceptance manifest binding changed")
    check(acceptance["next_stage_authorized"] is False,
          "G1 acceptance authorized G3")
    check(sha256(G1_MANIFEST) == EXPECTED_G1_MANIFEST_SHA,
          "G1 source manifest changed")
    g1 = load(G1_MANIFEST)
    excluded = set(g1["excluded_self_referential"])
    for relative, record in g1["run_files"].items():
        if relative in excluded:
            continue
        path = G1_RUN / relative
        check(path.is_file() and sha256(path) == record["sha256"],
              "accepted G1 run file changed: " + relative)
    g1_snapshot = load(G1_PROTECTED)
    check(sha256(G1_PROTECTED) == EXPECTED_G1_AFTER_SHA,
          "G1 protected_after changed")
    check(g1_snapshot["protected_tree_sha256"] == EXPECTED_G1_TREE,
          "G1 protected tree changed")
    check(canonical_tree(g1_snapshot["protected_files"]) == EXPECTED_G1_TREE,
          "G1 protected snapshot content changed")
    check(protection["g1_evidence"]["run_files_all_match"] is True,
          "protection diff did not confirm G1 run files")
    check(protection["g0_evidence"]["unchanged_since_g1_acceptance"] is True,
          "protection diff did not confirm G0 preservation")

    old = {row["path"]: row for row in g1_snapshot["protected_files"]}
    roots = g1_snapshot["protected_paths"]
    current_rows = []
    for raw_root in roots:
        root = REPO_ROOT / raw_root
        if not root.exists():
            continue
        candidates = [root] if (
            root.is_file() or root.is_symlink()) else sorted(
            path for path in root.rglob("*")
            if path.is_file() or path.is_symlink())
        for path in candidates:
            if path.name == "virtual_a_baseline.py":
                continue
            parts = set(path.relative_to(REPO_ROOT).parts)
            if parts & {"__pycache__"} or path.suffix in (".pyc", ".pyo"):
                continue
            relative = path.relative_to(REPO_ROOT).as_posix()
            if path.is_symlink():
                current_rows.append({
                    "path": relative,
                    "type": "symlink",
                    "sha256": hashlib.sha256(
                        os.fsencode(os.readlink(path))).hexdigest(),
                    "size_bytes": 0,
                })
            else:
                current_rows.append({
                    "path": relative,
                    "type": "file",
                    "sha256": sha256(path),
                    "size_bytes": path.stat().st_size,
                })
    current = {row["path"]: row for row in current_rows}
    added = sorted(set(current) - set(old))
    removed = sorted(set(old) - set(current))
    changed = sorted(
        path for path in set(old) & set(current)
        if (old[path]["type"], old[path]["sha256"], old[path]["size_bytes"])
        != (current[path]["type"], current[path]["sha256"],
            current[path]["size_bytes"]))
    check(added == sorted(protection["authorization"]["allowed_new_files"]),
          "current protected additions differ: {}".format(added))
    check(changed == sorted(EXPECTED_CHANGED),
          "current protected changes differ: {}".format(changed))
    check(not removed, "current protected removals exist: {}".format(removed))
    unchanged = sorted((set(old) & set(current)) - set(changed))
    check(canonical_tree([old[path] for path in unchanged]) ==
          canonical_tree([current[path] for path in unchanged]),
          "unchanged protected content differs beyond the whitelist")
    check(protection["current"]["full_protected_tree_sha256"] ==
          canonical_tree(current_rows),
          "protection diff full tree does not match disk")
    check(MANIFEST["protected"]["current_full_protected_tree_sha256"] ==
          canonical_tree(current_rows),
          "manifest protected tree does not match disk")
    check(MANIFEST["protected"]["comparison_sha256"] ==
          sha256(RUN / "analysis/protection_diff_vs_g1_accepted.json"),
          "manifest comparison hash changed")
    check(MANIFEST["protected"]["only_authorized_changes"] is True,
          "manifest authorization flag changed")
    check(MANIFEST["protected"]["g1_acceptance_status"] == "ACCEPTED",
          "manifest G1 acceptance status changed")
    check(MANIFEST["protected"]["g1_acceptance_sha256"] ==
          EXPECTED_G1_ACCEPTANCE_SHA, "manifest G1 acceptance hash changed")
    check(MANIFEST["protected"]["g1_manifest_sha256"] ==
          EXPECTED_G1_MANIFEST_SHA, "manifest G1 manifest hash changed")
    check(MANIFEST["protected"]["g1_protected_after_sha256"] ==
          EXPECTED_G1_AFTER_SHA, "manifest G1 snapshot hash changed")
    check(MANIFEST["protected"]["g1_manifest_run_files_all_match"] is True,
          "manifest G1 run-file confirmation changed")
    check(MANIFEST["protected"]["g1_manifest_run_files_checked"] == 47,
          "manifest G1 run-file count changed")
    check(MANIFEST["protected"]["g0_file_count"] == 3120,
          "manifest G0 file count changed")
    check(MANIFEST["protected"]["g0_unchanged"] is True,
          "manifest G0 preservation changed")
    check(MANIFEST["protected"]["g1_acceptance_next_stage_authorized"] is False,
          "manifest G1 authorization changed")


def test_log(name):
    path = RUN / "logs" / (name + ".log")
    text = path.read_text(encoding="utf-8")
    import re
    ran = re.search(r"Ran (\d+) tests? in ([0-9.]+)s", text)
    check(ran is not None, "unparseable test log: " + name)
    if ran is None:
        return None
    check(re.search(r"^OK(?: \(|$)", text, re.M) is not None,
          "test log not OK: " + name)
    check(int(ran.group(1)) == EXPECTED_TEST_COUNTS[name],
          "test count changed: " + name)
    return {
        "count": int(ran.group(1)),
        "wall_time_s": float(ran.group(2)),
        "cases": set(re.findall(
            r"^(\w+) \([\w.]+\) \.\.\. ok$", text, re.M)),
    }


def verify_results():
    results = load(RUN / "results.json")
    summary = load(RUN / "analysis" / "acceptance_summary.json")
    benchmark = load(RUN / "benchmark" / "summary.json")
    records = load(RUN / "benchmark" / "results.json")
    frozen = load(RUN / "frozen" / "frozen_manifest.json")
    attempts = load(RUN / "analysis" / "attempts.json")
    frozen_requests = {row["name"]: row for row in frozen["requests"]}
    check(set(frozen_requests) == set(NAMES),
          "frozen request set changed")
    check(all(row["benchmark_request_summary_matches_frozen"] is True
              for row in frozen_requests.values()),
          "a frozen request summary did not match the benchmark")
    check(attempts["benchmark_results_sha256"] ==
          sha256(RUN / "benchmark" / "results.json"),
          "attempt register benchmark binding changed")
    check(attempts["config"] == frozen["planner_config"]["frozen_values"],
          "attempt register config differs from frozen config")
    check(set(attempts["requests"]) == set(NAMES),
          "attempt register request set changed")
    for name in NAMES:
        frozen_row = frozen_requests[name]
        record = records[name]
        check(frozen_row["request_sha256"] == record["request_sha256"],
              "frozen request hash differs from benchmark: " + name)
        check(results["requests"][name]["request_sha256"] ==
              record["request_sha256"],
              "results request hash differs from benchmark: " + name)
        check(attempts["requests"][name]["request_sha256"] ==
              record["request_sha256"],
              "attempt request hash differs from benchmark: " + name)
    check(results["status"] == "PASS_PENDING_CODEX_RECHECK",
          "results status changed")
    check(results["codex_acceptance"] == "NOT_RUN_NOT_CLAIMED",
          "results self-claim acceptance")
    check(results["attempts"]["sha256"] ==
          sha256(RUN / "analysis/attempts.json"),
          "attempt register hash changed")
    check(results["benchmark"]["sha256"] ==
          sha256(RUN / "benchmark" / "summary.json"),
          "benchmark summary hash changed")
    check(results["benchmark"]["results_sha256"] ==
          sha256(RUN / "benchmark" / "results.json"),
          "benchmark results hash changed")
    check(results["frozen"]["manifest_sha256"] ==
          sha256(RUN / "frozen/frozen_manifest.json"),
          "frozen manifest hash changed")
    check(results["attempts"]["count"] == 48,
          "results attempt count changed")
    check(results["benchmark"]["checks"] == EXPECTED_CHECK_COUNT,
          "results benchmark check count changed")
    check(results["benchmark"]["failed_checks"] == [],
          "results record failed benchmark checks")
    check(results["benchmark"]["status"] == "PASS",
          "results benchmark status changed")
    check(results["frozen"]["planner_config_is_default"] is True,
          "results did not record default config")
    check(results["frozen"]["reference_profiles_passed_to_solve"] is False,
          "results recorded reference profile as a solve input")
    check(results["scope"]["executable_field"] == "NOT_PRESENT",
          "results claim an executable field")
    check(results["scope"]["output_status"] == "GEOMETRY_FEASIBLE only",
          "results scope status changed")
    check(results["request_names"] == list(NAMES),
          "results request order changed")
    check(frozen["planner_config"]["reconstructed_equals_PlannerConfig"] is True,
          "frozen PlannerConfig equality was not proven")
    check(frozen["planner_config"]["frozen_dict_equals_default_dict"] is True,
          "frozen config dict equality was not proven")
    check(frozen["planner_config"]["frozen_sha256_canonical"] ==
          summary["frozen_config"]["sha256_canonical"],
          "frozen config canonical hash differs")
    check(frozen["request_count"] == 8, "frozen request count changed")
    check(frozen["files"]["open_requests.json"]["frozen_sha256"] ==
          MANIFEST["source_files"][
              "tests/fixtures/virtual_a/open_requests.json"]["sha256"],
          "frozen request copy differs from source fixture")
    check(frozen["files"]["existence_reference.json"]["frozen_sha256"] ==
          MANIFEST["source_files"][
              "tests/fixtures/virtual_a/existence_reference.json"]["sha256"],
          "frozen reference copy differs from source fixture")
    check(benchmark["status"] == "PASS", "benchmark is not PASS")
    check(len(benchmark["checks"]) == EXPECTED_CHECK_COUNT,
          "benchmark check count changed")
    check(all(check_row["pass"] for check_row in benchmark["checks"]),
          "benchmark has a failed check")
    check(hashlib.sha256(json.dumps(
        benchmark["checks"], sort_keys=True, separators=(",", ":"),
        ensure_ascii=False).encode("utf-8")).hexdigest() ==
        summary["benchmark"]["checks_sha256"],
        "benchmark checks canonical hash changed")
    check(summary["all_requests_pass"] is True,
          "acceptance is not all-pass")
    check(summary["all_audits_passed"] is True,
          "not all audits passed")
    check(summary["all_requests_status"] == ["GEOMETRY_FEASIBLE"],
          "unexpected solve statuses")
    check(summary["deadline"]["any_deadline_exhausted"] is False,
          "a deadline was exhausted")

    max_position = 0.0
    max_heading = 0.0
    max_rigidity = 0.0
    max_position_budget = 0.0
    max_angle_budget = 0.0
    max_c_all = 0.0
    max_wheel_all = 0.0
    for name in NAMES:
        record = records[name]
        result = record["result"]
        audit = result["audit"]
        goal = record["request"]["goal_qf"]
        endpoint = record["endpoint"]
        position = math.hypot(endpoint["x"] - goal["x"],
                              endpoint["y"] - goal["y"])
        heading = abs(math.atan2(
            math.sin(endpoint["psi"] - goal["psi"]),
            math.cos(endpoint["psi"] - goal["psi"])))
        max_position = max(max_position, position)
        max_heading = max(max_heading, heading)
        check(position <= 0.005, "position tolerance failed: " + name)
        check(heading <= 0.003490658503988659,
              "heading tolerance failed: " + name)
        check(result["status"] == "GEOMETRY_FEASIBLE",
              "status changed: " + name)
        check(audit["passed"] is True, "audit not passed: " + name)
        check(audit["request_sha256"] == record["request_sha256"],
              "audit request digest mismatch: " + name)
        check(result["deadline_exhausted"] is False,
              "deadline exhausted: " + name)
        check(result["wall_time_s"] < 30.0, "solve overran budget: " + name)
        check(len(result["attempts"]) == 6, "attempt count changed: " + name)
        for bench_attempt, register_attempt in zip(
                result["attempts"], attempts["requests"][name]["attempts"]):
            check(set(bench_attempt) - set(register_attempt) == {"detail"},
                  "attempt register fields changed: " + name)
            check(all(register_attempt[key] == bench_attempt[key]
                      for key in register_attempt),
                  "attempt register differs from benchmark: " + name)
        check(record["initial_guesses"] ==
              attempts["requests"][name]["initial_guesses"],
              "initial guesses differ from attempt register: " + name)
        check(summary["per_request"][name]["status"] ==
              "GEOMETRY_FEASIBLE", "summary status changed: " + name)
        check(summary["per_request"][name]["attempt_count"] == 6,
              "summary attempt count changed: " + name)
        check(summary["per_request"][name]["attempt_solvers"] == ["ipopt"],
              "summary attempt solvers changed: " + name)
        check(summary["per_request"][name]["attempt_statuses"] ==
              ["SOLVER_SUCCESS_AUDITED"],
              "summary attempt statuses changed: " + name)
        check(summary["per_request"][name]["iterations"] ==
              [attempt["iterations"] for attempt in result["attempts"]],
              "summary iterations differ: " + name)
        check(summary["per_request"][name]["audit_passed"] is True,
              "summary audit flag changed: " + name)
        check(summary["per_request"][name]["all_segments_confirmed"] is True,
              "summary segment confirmation changed: " + name)
        check(summary["per_request"][name]["wall_time_s"] ==
              result["wall_time_s"], "summary wall differs: " + name)
        check(summary["per_request"][name]["endpoint_position_error_m"] ==
              position, "summary position error differs: " + name)
        check(summary["per_request"][name]["endpoint_heading_error_rad"] ==
              heading, "summary heading error differs: " + name)
        for attempt in result["attempts"]:
            check(attempt["status"] == "SOLVER_SUCCESS_AUDITED",
                  "attempt status changed: " + name)
            check(attempt["solver"] == "ipopt",
                  "attempt solver changed: " + name)
            check(attempt["candidate_audit"]["passed"] is True,
                  "candidate audit failed: " + name)
            check(attempt["iterations"] <= 120,
                  "attempt exceeded max_iterations: " + name)
        rigidity = audit["metrics"]["sample_audit"][
            "max_pairwise_rigidity_error_m"]
        max_rigidity = max(max_rigidity, rigidity)
        check(rigidity <= 1e-6, "rigidity tolerance failed: " + name)
        position_budget = audit["integration"]["position_budget_m"]
        angle_budget = audit["integration"]["angle_budget_rad"]
        max_position_budget = max(max_position_budget, position_budget)
        max_angle_budget = max(max_angle_budget, angle_budget)
        check(position_budget <= 1e-7, "position budget failed: " + name)
        check(angle_budget <= 1e-9, "angle budget failed: " + name)
        segments = audit["whole_segment_bounds"]
        check(len(segments) == 8, "segment count changed: " + name)
        max_c = max(value for segment in segments
                    for value in segment["max_abs_C_by_vehicle"].values())
        max_wheel = max(segment["max_static_wheel_angle_rad"]
                        for segment in segments)
        max_c_all = max(max_c_all, max_c)
        max_wheel_all = max(max_wheel_all, max_wheel)
        check(max_c <= 1.0 / record["request"]["limits"]["r_min_ackermann"],
              "curvature limit failed: " + name)
        check(max_wheel <= record["request"]["limits"]["steer_angle_max"],
              "wheel limit failed: " + name)
        check(all(segment["passed"] for segment in segments),
              "a segment envelope is unconfirmed: " + name)
        check(summary["per_request"][name]["max_abs_C"] == max_c,
              "summary max C differs: " + name)
        check(summary["per_request"][name]["max_static_wheel_angle_rad"] ==
              max_wheel, "summary wheel max differs: " + name)
        if name == "STRAIGHT_POLICY":
            terminal = record["profile"]["knots"][-1]
            check(abs(terminal["value"]) <= 1e-9,
                  "STRAIGHT terminal kappa is not zero")
            check(abs(terminal["derivative"]) <= 1e-9,
                  "STRAIGHT terminal eta is not zero")
    check(max_position == summary["maximums"]["endpoint_position_error_m"],
          "maximum position error summary changed")
    check(max_heading == summary["maximums"]["endpoint_heading_error_rad"],
          "maximum heading error summary changed")
    check(max_rigidity == summary["maximums"]["rigidity_error_m"],
          "maximum rigidity summary changed")
    check(max_position_budget == summary["maximums"]["position_budget_m"],
          "maximum position budget summary changed")
    check(max_angle_budget == summary["maximums"]["angle_budget_rad"],
          "maximum angle budget summary changed")
    check(max_c_all == summary["maximums"]["max_abs_C"],
          "maximum curvature summary changed")
    check(max_wheel_all == summary["maximums"]["max_static_wheel_angle_rad"],
          "maximum wheel summary changed")
    check(summary["deadline"]["max_observed_solve_wall_time_s"] ==
          max(records[name]["result"]["wall_time_s"] for name in NAMES),
          "maximum wall summary changed")
    check(summary["deadline"]["timeout_point"] == "NOT_TRIGGERED",
          "timeout point changed")
    check(summary["frozen_config"]["equals_PlannerConfig_default"] is True,
          "summary config equality changed")
    check(summary["frozen_config"]["sha256_canonical"] ==
          frozen["planner_config"]["frozen_sha256_canonical"],
          "summary config hash differs")

    parsed_logs = {}
    for name, expected in EXPECTED_TEST_COUNTS.items():
        parsed = test_log(name)
        parsed_logs[name] = parsed
        if parsed is None:
            continue
        check(summary["tests"][name]["count"] == expected,
              "summary test count changed: " + name)
        check(summary["tests"][name]["status"] == "OK",
              "summary test status changed: " + name)
        check(summary["tests"][name]["wall_time_s"] ==
              parsed["wall_time_s"], "summary test wall changed: " + name)
        check(results["tests"][name]["count"] == expected,
              "results test count changed: " + name)
        check(results["tests"][name]["status"] == "OK",
              "results test status changed: " + name)
        check(results["tests"][name]["log_sha256"] ==
              sha256(RUN / "logs" / (name + ".log")),
              "results test-log hash changed: " + name)
    coverage = summary["negative_fixture_coverage"]
    check(len(coverage) == EXPECTED_NEGATIVE_COVERAGE_COUNT,
          "negative coverage count changed")
    check(REQUIRED_NEGATIVE_TESTS <=
          {entry["test"] for entry in coverage},
          "a required negative fixture is not covered")
    check(results["negative"]["coverage"] == coverage,
          "results negative coverage differs from the summary")
    for entry in coverage:
        check(entry["test_outcome"] == "ok",
              "negative fixture did not pass: " + entry["test"])
        check(entry["log_sha256"] == sha256(RUN / entry["log_path"]),
              "negative coverage log hash changed: " + entry["test"])
        parsed = parsed_logs.get(entry["log"])
        check(parsed is not None and entry["test"] in parsed["cases"],
              "negative test missing from its log: " + entry["test"])
    check(attempts["schema"] == "virtual_rigid_a.g2.attempts.v1",
          "attempt register schema changed")
    check(sum(record["attempt_count"]
              for record in attempts["requests"].values()) == 48,
          "attempt register count changed")
    for name in NAMES:
        check(len(attempts["requests"][name]["initial_guesses"]) == 6,
              "initial guess count changed: " + name)
        check(attempts["requests"][name]["attempt_count"] == 6,
              "attempt register count changed: " + name)
        check(len(attempts["requests"][name]["attempts"]) == 6,
              "attempt register list count changed: " + name)


def verify_git():
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD"],
        cwd=str(REPO_ROOT), check=False).returncode
    check(head == BASELINE_HEAD, "git HEAD changed")
    check(branch == "feature/virtual-rigid-formation-a",
          "git branch changed")
    check(ancestor == 0, "baseline is no longer an ancestor")
    check(not staged, "staged changes are present")
    check(MANIFEST["git"]["committed"] == "NO", "manifest claims a commit")
    check(MANIFEST["git"]["pushed"] == "NO", "manifest claims a push")
    check(MANIFEST["git"]["branch_created"] == "NO",
          "manifest claims a new branch")
    check(MANIFEST["git"]["head"] == BASELINE_HEAD,
          "manifest git head changed")
    check(MANIFEST["git"]["branch"] == "feature/virtual-rigid-formation-a",
          "manifest git branch changed")
    check(MANIFEST["git"]["baseline_is_ancestor"] is True,
          "manifest baseline ancestry changed")
    check(MANIFEST["git"]["staged_diff_sha256"] ==
          hashlib.sha256(b"").hexdigest(),
          "manifest records a non-empty staged diff")
    current_status = set(git("status", "--short").splitlines())
    recorded_status = set(MANIFEST["git"]["status_short"])
    check(recorded_status == current_status,
          "git status differs from the manifest: missing={} extra={}".format(
              sorted(recorded_status - current_status),
              sorted(current_status - recorded_status)))


def main():
    print(json.dumps({
        "schema": "virtual_rigid_a.g2.verification.v1",
        "started_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "manifest": str(MANIFEST_PATH.relative_to(REPO_ROOT)),
        "manifest_sha256": sha256(MANIFEST_PATH),
    }, indent=2, sort_keys=True))
    verify_header()
    verify_sources()
    verify_run_files()
    verify_protected()
    verify_results()
    verify_git()
    result = {
        "status": "OK" if not FAILURES else "FAILED",
        "failures": FAILURES,
        "checks_recomputed": {
            "source_files": MANIFEST["source_file_count"],
            "run_files": MANIFEST["run_file_count"],
            "reference_documents": MANIFEST["reference_document_count"],
            "protected_files_current": MANIFEST["protected"][
                "current_full_protected_tree_sha256"],
            "protected_tree_g1": EXPECTED_G1_TREE,
            "protected_tree_g0": EXPECTED_G0_TREE,
            "requests": len(NAMES),
            "attempts": 48,
            "benchmark_checks": EXPECTED_CHECK_COUNT,
        },
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if not FAILURES else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())