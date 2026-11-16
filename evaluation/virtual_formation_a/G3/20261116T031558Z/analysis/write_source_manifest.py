"""Write the G3 rework source-and-evidence manifest.

Binds the accepted G2 source manifest and protected tree, the G3 source/test/
task files including the rework order, every evidence-run file except the
three self-referential artifacts, the protection diff, the independent
numerical interval-coverage evidence, the benchmark/model results and the
truthful legacy regression record.  The independent verifier recomputes all of
it.
"""
import datetime
import hashlib
import json
import math
import re
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
RUN_ID = RUN.name
SCHEMA = "virtual_rigid_a.g3.rework.source_manifest.v1"
STAGE = "G3"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
G1_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G1/20261115T152858Z"
G1_PROTECTED = G1_RUN / "legacy/protected_after.json"
G2_ACCEPTANCE = (REPO_ROOT / "evaluation/virtual_formation_a/G2" /
                 "codex_acceptance_rework_03/CODEX_ACCEPTANCE.json")
G2_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G2/20261115T174526Z"
G2_MANIFEST = G2_RUN / "source_manifest.json"
G2_PROTECTION = G2_RUN / "analysis/protection_diff_vs_g1_accepted.json"
EXPECTED_G1_TREE = (
    "910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519")
EXPECTED_G1_AFTER_SHA = (
    "c9e4c874c3e7d0dc05a1850d74febd28ec7475a232b6f8f98d3e1c11d2892470")
EXPECTED_G2_MANIFEST_SHA = (
    "852cb44615321ed69613f6dfbe6ae6b2b79da5a17563c38537e899209dc90886")
EXPECTED_G2_TREE = (
    "fa1b0596f7b9ecef616512785ab65ef2b93884dfde3a9c2a9794a9f7ac222f67")
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
G3_SOURCES = [
    PACKAGE + "timing.py",
    PACKAGE + "trajectory.py",
    "docs/virtual_formation_a/CLAUDE_G3_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G3_TASK.md",
    "tests/test_virtual_a_timing.py",
    "tests/test_virtual_a_trajectory.py",
]
REFERENCE_DOCUMENTS = [
    "docs/VIRTUAL_FORMATION_A_FR06_TECHNICAL_REVIEW_20261115.md",
]
EXCLUSIONS = [
    "source_manifest.json",
    "logs/source_manifest_generation.log",
    "logs/source_manifest_verification.log",
]
EXPECTED_CHANGED = sorted([
    PACKAGE + "audit.py",
    PACKAGE + "bounds.py",
    PACKAGE + "contracts.py",
    "tests/test_virtual_a_audit.py",
    "tools/virtual_a_benchmark.py",
])
EXPECTED_TEST_COUNTS = {"audit": 51, "timing": 18, "trajectory": 23}
EXPECTED_CHECK_COUNT = 151
EXPECTED_SOURCE_COUNT = 31
EXPECTED_MOTION_AUDIT_CHECKS = 909
EXPECTED_HOLD_AUDIT_CHECKS = 16
EXPECTED_CERTIFICATE_LEAVES = 64
EXPECTED_POINTWISE_VEHICLE_CHECKS = 3 * EXPECTED_CERTIFICATE_LEAVES
EXPECTED_CASES = [
    "E1", "E2", "E3", "E4", "E5", "E6", "NO_MOTION_HOLD",
    "STRAIGHT_LINE", "STRAIGHT_POLICY"]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path):
    path = Path(path)
    return {"bytes": path.stat().st_size, "sha256": sha256(path)}


def canonical_protected_tree(entries):
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


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def verify_g2_evidence(g2_manifest, g2_protection, g1_snapshot):
    acceptance = load(G2_ACCEPTANCE)
    if acceptance.get("status") != "ACCEPTED":
        raise SystemExit("G2 acceptance is not ACCEPTED")
    if acceptance.get("source_manifest_sha256") != EXPECTED_G2_MANIFEST_SHA:
        raise SystemExit("G2 acceptance does not bind the G2 manifest")
    if acceptance.get("next_stage_authorized") is not False:
        raise SystemExit("G2 acceptance unexpectedly authorizes G3")
    if sha256(G2_MANIFEST) != EXPECTED_G2_MANIFEST_SHA:
        raise SystemExit("accepted G2 source manifest changed")
    if g2_manifest["protected"][
            "current_full_protected_tree_sha256"] != EXPECTED_G2_TREE:
        raise SystemExit("G2 manifest protected tree changed")
    if g2_protection["current"][
            "full_protected_tree_sha256"] != EXPECTED_G2_TREE:
        raise SystemExit("G2 protection diff tree changed")
    for relative, record in acceptance["source_sha256"].items():
        source = g2_manifest["source_files"].get(relative)
        if source is None or source["sha256"] != record:
            raise SystemExit("G2 acceptance source hash is unbound")
    excluded = set(g2_manifest.get("excluded_self_referential", []))
    checked = {}
    for relative, record in sorted(g2_manifest["run_files"].items()):
        if relative in excluded:
            continue
        path = G2_RUN / relative
        if not path.is_file() or sha256(path) != record["sha256"]:
            raise SystemExit("accepted G2 run file changed: " + relative)
        checked[relative] = record
    g2_rows = {row["path"]: dict(row) for row in
               g1_snapshot["protected_files"]}
    for row in g2_protection["added_vs_g1_accepted"]:
        g2_rows[row["path"]] = {
            "path": row["path"], "type": "file",
            "sha256": row["sha256"], "size_bytes": row["size_bytes"]}
    for row in g2_protection["changed_vs_g1_accepted"]:
        old = g2_rows[row["path"]]
        g2_rows[row["path"]] = {
            "path": row["path"], "type": old["type"],
            "sha256": row["g2_sha256"],
            "size_bytes": row["g2_size_bytes"]}
    if canonical_protected_tree(g2_rows.values()) != EXPECTED_G2_TREE:
        raise SystemExit("G2 protected-tree reconstruction failed")
    return acceptance, checked, g2_rows


def parse_test(name):
    text = (RUN / "logs" / (name + ".log")).read_text(encoding="utf-8")
    match = re.search(r"Ran (\d+) tests? in ([0-9.]+)s", text)
    if match is None or int(match.group(1)) != EXPECTED_TEST_COUNTS[name] or \
            text.rstrip().splitlines()[-1].strip() != "OK":
        raise SystemExit("{} test log is not the expected OK run".format(name))
    return {"count": int(match.group(1)),
            "wall_time_s": float(match.group(2)),
            "log_sha256": sha256(RUN / "logs" / (name + ".log"))}


def main():
    for required in ("CLAUDE_REPORT.md", "COMMANDS.md", "environment.json",
                     "results.json",
                     "analysis/protection_diff_vs_g2_accepted.json",
                     "analysis/g3_rework_numerics.json",
                     "analysis/verify_source_manifest.py",
                     "logs/exit_codes.log",
                     "logs/g3_rework_numerics.log",
                     "logs/protection_diff.log",
                     "logs/assemble_evidence.log",
                     "logs/write_environment.log"):
        if not (RUN / required).is_file():
            raise SystemExit("missing evidence before manifest: " + required)
    pycache = [path for path in RUN.rglob("*")
               if path.name == "__pycache__"
               or path.suffix in (".pyc", ".pyo")]
    if pycache:
        raise SystemExit("generated bytecode exists under the run")
    source_pycache = [path for path in REPO_ROOT.rglob("*")
                      if path.name == "__pycache__"
                      and "evaluation" not in path.parts]
    if source_pycache:
        raise SystemExit("generated bytecode exists in source")

    g1_snapshot = load(G1_PROTECTED)
    if sha256(G1_PROTECTED) != EXPECTED_G1_AFTER_SHA:
        raise SystemExit("accepted G1 protected_after.json changed")
    if g1_snapshot["protected_tree_sha256"] != EXPECTED_G1_TREE:
        raise SystemExit("G1 tree changed")
    if canonical_protected_tree(g1_snapshot["protected_files"]) != \
            EXPECTED_G1_TREE:
        raise SystemExit("G1 protected snapshot is inconsistent")

    g2_manifest = load(G2_MANIFEST)
    g2_protection = load(G2_PROTECTION)
    acceptance, g2_checked, g2_rows = verify_g2_evidence(
        g2_manifest, g2_protection, g1_snapshot)

    g2_sources = dict(g2_manifest["source_files"])
    source_paths = sorted(set(g2_sources) | set(G3_SOURCES))
    if len(source_paths) != EXPECTED_SOURCE_COUNT:
        raise SystemExit("expected {} source files, got {}".format(
            EXPECTED_SOURCE_COUNT, len(source_paths)))
    source_files = {
        relative: file_record(REPO_ROOT / relative)
        for relative in source_paths}
    changed_sources = {}
    for relative in sorted(g2_sources):
        old_hash = g2_sources[relative]["sha256"]
        new_hash = source_files[relative]["sha256"]
        changed_sources[relative] = {
            "g2_sha256": old_hash,
            "g3_sha256": new_hash,
            "changed": old_hash != new_hash,
        }
    actually_changed = sorted(
        path for path, record in changed_sources.items() if record["changed"])
    if actually_changed != EXPECTED_CHANGED:
        raise SystemExit("unexpected changes to accepted G2 sources: "
                         "{}".format(actually_changed))
    for relative in G3_SOURCES:
        if relative in g2_sources:
            raise SystemExit("new G3 source was already accepted: " + relative)
    reference_files = {
        relative: file_record(REPO_ROOT / relative)
        for relative in REFERENCE_DOCUMENTS}

    protection_path = (RUN / "analysis" /
                       "protection_diff_vs_g2_accepted.json")
    protection = load(protection_path)
    if not protection["authorization"]["only_authorized_changes"]:
        raise SystemExit("protection diff is not exactly authorized")
    if protection["removed_vs_g2_accepted"]:
        raise SystemExit("protected removals are present")
    added = sorted(row["path"] for row in
                   protection["added_vs_g2_accepted"])
    changed = sorted(row["path"] for row in
                     protection["changed_vs_g2_accepted"])
    if added != sorted(protection["authorization"]["allowed_new_files"]):
        raise SystemExit("protected additions differ from the G3 whitelist")
    if changed != EXPECTED_CHANGED:
        raise SystemExit("protected changes differ from the G3 whitelist")
    if protection["current"]["unchanged_common_tree_matches"] is not True:
        raise SystemExit("common protected tree changed")
    if protection["g2_evidence"]["protected_tree_sha256"] != EXPECTED_G2_TREE:
        raise SystemExit("protection diff G2 tree changed")

    results = load(RUN / "results.json")
    if results["status"] != "PASS_PENDING_CODEX_RECHECK":
        raise SystemExit("results status is not pending Codex recheck")
    if results["codex_acceptance"] != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("results self-claim Codex acceptance")
    scope = results["scope"]
    if scope["timing_and_dynamics_proven"] is not True:
        raise SystemExit("results did not record the G3 proof")
    for key in ("collision_checked", "executable", "map_used", "ros_used",
                "gazebo_used", "hardware_used", "time_optimal", "g4_entered"):
        if scope[key] is not False:
            raise SystemExit("results scope claims {}".format(key))
    tests = {name: parse_test(name) for name in EXPECTED_TEST_COUNTS}
    for name, record in tests.items():
        if results["tests"][name]["count"] != record["count"] or \
                results["tests"][name]["status"] != "OK":
            raise SystemExit("results test record changed: " + name)
    if results["benchmark"]["status"] != "PASS" or \
            results["benchmark"]["check_count"] != EXPECTED_CHECK_COUNT or \
            results["benchmark"]["failed"]:
        raise SystemExit("benchmark record is not 151/151 PASS")
    if results["aggregate"]["benchmark_check_count"] != EXPECTED_CHECK_COUNT:
        raise SystemExit("aggregate benchmark count changed")
    if results["aggregate"]["pointwise_leaf_count"] != \
            EXPECTED_CERTIFICATE_LEAVES * 8:
        raise SystemExit("aggregate pointwise leaf count changed")
    if results["aggregate"]["pointwise_vehicle_count"] != \
            EXPECTED_POINTWISE_VEHICLE_CHECKS * 8:
        raise SystemExit("aggregate pointwise vehicle count changed")
    for key in ("max_pointwise_jerk_ratio",
                "max_pointwise_component_ratio",
                "max_pointwise_p_sss_ratio"):
        value = results["aggregate"][key]
        if not isinstance(value, float) or not (0.0 <= value <= 1.0):
            raise SystemExit("pointwise containment ratio failed: " + key)
    coverage = results["interval_coverage"]
    if coverage["degenerate_counterexample"][
            "expected_p_sss_norm"] != 0.10677546534667971:
        raise SystemExit("Codex counterexample norm changed")
    if coverage["degenerate_counterexample"]["timing_jerk"] < \
            coverage["degenerate_counterexample"]["expected_p_sss_norm"] or \
            coverage["degenerate_counterexample"]["audit_jerk"] < \
            coverage["degenerate_counterexample"]["expected_p_sss_norm"]:
        raise SystemExit("Codex counterexample is not covered")
    if coverage["narrow_interval_case_count"] != 24 or \
            coverage["narrow_interval_failure_count"] != 0:
        raise SystemExit("narrow interval coverage changed")
    signs = set()
    front_nonzero = 0
    for path in coverage["positive_paths"]:
        if not path["all_contained"]:
            raise SystemExit("positive path containment failed")
        if path["front_nonzero_k_eta_samples"] <= 0:
            raise SystemExit("front vehicle has no nonzero k*eta sample")
        front_nonzero += path["front_nonzero_k_eta_samples"]
        for pair in path["front_sign_pairs"]:
            signs.add((int(pair[0]), int(pair[1])))
    if signs != {(-1, -1), (-1, 1), (1, -1), (1, 1)}:
        raise SystemExit("positive paths do not cover all four sign pairs")
    if coverage["jerk_overlimit_negative"]["audit_reason"] != \
            "DYNAMIC_LIMIT":
        raise SystemExit("negative jerk audit did not reject")
    if coverage["jerk_overlimit_negative"]["first_failure"][
            "value"] <= coverage["jerk_overlimit_negative"]["limit"]["jerk_max"]:
        raise SystemExit("negative jerk actual does not exceed its limit")
    if coverage["jerk_overlimit_negative"][
            "interior_overlimit_samples"] <= 0:
        raise SystemExit("negative jerk evidence lacks an interior witness")
    if coverage["broad_interval_fail_closed"]["coarse"]["reason"] != \
            "BOUND_UNCONFIRMED":
        raise SystemExit("coarse interval did not fail closed")
    if coverage["broad_interval_fail_closed"]["refined"]["passed"] is not True:
        raise SystemExit("refined interval did not confirm the bound")
    numerics = load(RUN / coverage["numerics_location"])
    if hashlib.sha256(
            (RUN / coverage["numerics_location"]).read_bytes()).hexdigest() != \
            coverage["numerics_sha256"]:
        raise SystemExit("numerics evidence digest changed")
    if numerics["schema"] != "virtual_rigid_a.g3.rework_numerics.v1":
        raise SystemExit("numerics evidence schema changed")
    if results["aggregate"]["dynamic_feasible_cases"] != 8 or \
            results["aggregate"]["no_motion_cases"] != 1:
        raise SystemExit("case status counts changed")
    if results["aggregate"]["short_duration_rejections"] != 8 or \
            results["aggregate"]["tamper_rejections"] != 8:
        raise SystemExit("short-duration/tamper rejection count changed")
    if [case["name"] for case in results["model_cases"]] != EXPECTED_CASES:
        raise SystemExit("model case list changed")
    for case in results["model_cases"]:
        audit = case["audit"]
        if case["name"] == "NO_MOTION_HOLD":
            if audit["check_count"] != EXPECTED_HOLD_AUDIT_CHECKS:
                raise SystemExit("hold audit check count changed")
            if audit.get("pointwise_interval_checks") is not None:
                raise SystemExit("hold audit has a pointwise certificate")
            continue
        if audit["check_count"] != EXPECTED_MOTION_AUDIT_CHECKS:
            raise SystemExit("motion audit check count changed")
        pointwise = audit["pointwise_interval_checks"]
        if pointwise["leaf_count"] != EXPECTED_CERTIFICATE_LEAVES or \
                pointwise["vehicle_count"] != \
                EXPECTED_POINTWISE_VEHICLE_CHECKS:
            raise SystemExit("pointwise leaf/vehicle count changed")
        for key in ("max_jerk_ratio", "max_component_ratio",
                    "max_p_sss_ratio"):
            if not (0.0 <= pointwise[key] <= 1.0):
                raise SystemExit("pointwise case ratio failed: " + key)
        if pointwise["worst"] is None or not (
                0.0 <= pointwise["worst"]["ratio"] <= 1.0):
            raise SystemExit("pointwise worst record changed")
        for node in audit["nodes"]["records"]:
            for side in ("left", "right"):
                record = node["pointwise_interval"][side]
                for key in ("jerk_ratio", "component_ratio",
                            "p_sss_ratio"):
                    if not (0.0 <= record[key] <= 1.0):
                        raise SystemExit("node pointwise ratio failed")
                if record["worst"] is None:
                    raise SystemExit("node pointwise worst missing")
    benchmark_results = load(RUN / "benchmark" / "results.json")
    for case in results["model_cases"]:
        name = case["name"]
        record = benchmark_results[name]
        if hashlib.sha256(
                record["model_json"].encode("utf-8")).hexdigest() != \
                record["model_sha256"]:
            raise SystemExit("case model digest changed: " + name)
        if case["model_sha256"] != record["model_sha256"] or \
                case["request_sha256"] != record["request_sha256"]:
            raise SystemExit("results model binding changed: " + name)

    benchmark = load(RUN / "benchmark" / "summary.json")
    if benchmark["status"] != "PASS" or len(benchmark["checks"]) != \
            EXPECTED_CHECK_COUNT or any(
                not check["pass"] for check in benchmark["checks"]):
        raise SystemExit("benchmark summary is not 151/151 PASS")
    if benchmark["codex_acceptance"] != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("benchmark self-claim Codex acceptance")

    legacy = load(RUN / "legacy" / "regression_results.json")
    statuses = {row["case_id"]: row["status"] for row in legacy["records"]}
    if legacy["status"] != "FAIL" or statuses != {
            "timing_continuous": "FAIL",
            "timing_formation": "PASS",
            "fr02_a_timing": "PASS",
            "fr02_a_trajectory": "PASS",
            "fr02_b_quality": "PASS"}:
        raise SystemExit("legacy regression record changed")
    gate = (RUN / "logs" / "legacy_unmodified_gate.log").read_text()
    if "UNKNOWN_WORKSPACE_MODIFICATIONS" not in gate:
        raise SystemExit("unmodified-gate failure missing")
    probe = (RUN / "logs" / "legacy_baseline_probe.log").read_text()
    if "KeyError: 'dynamic_peaks'" not in probe or "probe_exit=1" not in probe:
        raise SystemExit("pre-existing baseline probe missing")

    run_files = {}
    for path in sorted(RUN.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(RUN).as_posix()
        if relative in EXCLUSIONS:
            continue
        run_files[relative] = file_record(path)
    run_total_bytes = sum(record["bytes"] for record in run_files.values())

    status = git("status", "--short")
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD"],
        cwd=str(REPO_ROOT), check=False).returncode
    if head != BASELINE_HEAD or branch != "feature/virtual-rigid-formation-a":
        raise SystemExit("unexpected git HEAD or branch")
    if ancestor != 0:
        raise SystemExit("baseline is not an ancestor")
    if staged:
        raise SystemExit("staged changes are present")

    manifest = {
        "schema": SCHEMA,
        "stage": STAGE,
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN_ID,
        "repo_root": str(REPO_ROOT),
        "baseline_head": BASELINE_HEAD,
        "excluded_self_referential": EXCLUSIONS,
        "exclusion_reasons": {
            "source_manifest.json": "the manifest cannot hash itself",
            "logs/source_manifest_generation.log":
                "the generator writes this log while scanning the run",
            "logs/source_manifest_verification.log":
                "the verifier writes this log while checking the manifest",
        },
        "scope_note": (
            "G3 rework: corrected rigid-offset third-derivative sign in both "
            "interval implementations, common-time quintic trajectory and "
            "independent whole-domain dynamic audit only. DYNAMIC_FEASIBLE "
            "is not collision freedom, "
            "executability, time optimality, map feasibility, ROS/Gazebo or "
            "hardware acceptance. HEAD is unchanged and no commit, push, "
            "branch or G4 entry occurred."),
        "source_files": source_files,
        "source_file_count": len(source_files),
        "g3_source_files": sorted(G3_SOURCES),
        "reference_documents": reference_files,
        "changed_accepted_sources": changed_sources,
        "changed_accepted_source_list": actually_changed,
        "run_files": run_files,
        "run_file_count": len(run_files),
        "run_total_bytes": run_total_bytes,
        "git": {
            "head": head,
            "branch": branch,
            "baseline_head": BASELINE_HEAD,
            "baseline_is_ancestor": True,
            "committed": "NO",
            "pushed": "NO",
            "branch_created": "NO",
            "status_short": status.splitlines(),
            "staged_diff_sha256": hashlib.sha256(
                staged.encode("utf-8")).hexdigest(),
        },
        "protected": {
            "g1_protected_after_path":
                str(G1_PROTECTED.relative_to(REPO_ROOT)),
            "g1_protected_after_sha256": sha256(G1_PROTECTED),
            "g1_tree_sha256": g1_snapshot["protected_tree_sha256"],
            "g1_file_count": g1_snapshot["protected_file_count"],
            "g2_acceptance_path": str(G2_ACCEPTANCE.relative_to(REPO_ROOT)),
            "g2_acceptance_sha256": sha256(G2_ACCEPTANCE),
            "g2_acceptance_status": acceptance["status"],
            "g2_acceptance_next_stage_authorized":
                acceptance["next_stage_authorized"],
            "g2_run": str(G2_RUN.relative_to(REPO_ROOT)),
            "g2_manifest_sha256": sha256(G2_MANIFEST),
            "g2_manifest_run_files_checked": len(g2_checked),
            "g2_manifest_run_files_all_match": True,
            "g2_tree_sha256": EXPECTED_G2_TREE,
            "g2_file_count": len(g2_rows),
            "comparison_path": str(protection_path.relative_to(RUN)),
            "comparison_sha256": sha256(protection_path),
            "added_vs_g2_accepted": added,
            "changed_vs_g2_accepted": changed,
            "removed_vs_g2_accepted": protection["removed_vs_g2_accepted"],
            "only_authorized_changes": True,
            "current_full_protected_tree_sha256":
                protection["current"]["full_protected_tree_sha256"],
            "unchanged_common_tree_sha256":
                protection["current"][
                    "unchanged_common_tree_sha256_current"],
        },
        "results": {
            "status": results["status"],
            "codex_acceptance": results["codex_acceptance"],
            "model_case_count": len(results["model_cases"]),
            "model_cases": [case["name"] for case in results["model_cases"]],
            "benchmark_status": benchmark["status"],
            "benchmark_checks": len(benchmark["checks"]),
            "benchmark_failed": sum(
                not check["pass"] for check in benchmark["checks"]),
            "tests": {name: EXPECTED_TEST_COUNTS[name]
                      for name in EXPECTED_TEST_COUNTS},
            "motion_audit_check_count": EXPECTED_MOTION_AUDIT_CHECKS,
            "hold_audit_check_count": EXPECTED_HOLD_AUDIT_CHECKS,
            "certificate_leaves_per_motion_case":
                EXPECTED_CERTIFICATE_LEAVES,
            "pointwise_leaf_count":
                results["aggregate"]["pointwise_leaf_count"],
            "pointwise_vehicle_count":
                results["aggregate"]["pointwise_vehicle_count"],
            "max_pointwise_jerk_ratio":
                results["aggregate"]["max_pointwise_jerk_ratio"],
            "max_pointwise_component_ratio":
                results["aggregate"]["max_pointwise_component_ratio"],
            "max_pointwise_p_sss_ratio":
                results["aggregate"]["max_pointwise_p_sss_ratio"],
            "degenerate_counterexample_norm":
                coverage["degenerate_counterexample"][
                    "expected_p_sss_norm"],
            "narrow_interval_case_count":
                coverage["narrow_interval_case_count"],
            "narrow_interval_failure_count":
                coverage["narrow_interval_failure_count"],
            "positive_curvature_paths":
                [path["name"] for path in coverage["positive_paths"]],
            "positive_sign_pairs": sorted(
                [int(pair[0]), int(pair[1])] for path in
                coverage["positive_paths"] for pair in
                path["front_sign_pairs"]),
            "jerk_overlimit_audit_reason":
                coverage["jerk_overlimit_negative"]["audit_reason"],
            "jerk_overlimit_first_failure":
                coverage["jerk_overlimit_negative"]["first_failure"],
            "interior_overlimit_samples":
                coverage["jerk_overlimit_negative"][
                    "interior_overlimit_samples"],
            "broad_interval_coarse_reason":
                coverage["broad_interval_fail_closed"]["coarse"][
                    "reason"],
            "broad_interval_refined_leaves":
                coverage["broad_interval_fail_closed"]["refined"][
                    "leaves"],
            "numerics_location": coverage["numerics_location"],
            "numerics_sha256": coverage["numerics_sha256"],
            "dynamic_feasible_cases":
                results["aggregate"]["dynamic_feasible_cases"],
            "no_motion_cases": 1,
            "short_duration_rejections":
                results["aggregate"]["short_duration_rejections"],
            "tamper_rejections": results["aggregate"]["tamper_rejections"],
            "max_differential_errors":
                results["aggregate"]["max_differential_errors"],
            "max_time_inverse_error_s":
                results["aggregate"]["max_time_inverse_error_s"],
            "inverse_samples_per_case": 100,
            "legacy_status": legacy["status"],
            "legacy_passes": sorted(
                name for name, value in statuses.items() if value == "PASS"),
            "legacy_failure": "timing_continuous",
            "legacy_failure_pre_existing_probe": True,
            "collision_checked": False,
            "executable": False,
            "map_used": False,
            "ros_used": False,
            "time_optimal": False,
            "g4_entered": False,
            "nonclaims": results["nonclaims"],
        },
        "verification": {
            "script": "analysis/verify_source_manifest.py",
            "command": (
                "PYTHONDONTWRITEBYTECODE=1 python3 -B -I "
                "$RUN/analysis/verify_source_manifest.py"),
            "log": "logs/source_manifest_verification.log",
            "rule": (
                "Recompute every source and run-file SHA-256, reconstruct the "
                "accepted G2 protected tree from the accepted G1 snapshot and "
                "G2 diff, recheck the G3 protection whitelist, the 31 source "
                "hashes, all nine canonical model digests/request bindings, "
                "the 151 benchmark checks, 18/23/51 test logs, the 909/16 "
                "audit check counts, 512 pointwise leaf certificates and "
                "their 1536 vehicle containment ratios, the Codex degenerate "
                "counterexample, 24 narrow-interval cases, all four curvature/"
                "curvature-rate sign pairs, the independent negative and "
                "fail-closed witnesses, the 100-sample inverse and "
                "differential maxima, the truthful legacy FAIL and "
                "pristine-HEAD probe, and the git state."),
        },
    }
    target = RUN / "source_manifest.json"
    target.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "run_id": RUN_ID,
        "manifest": str(target.relative_to(REPO_ROOT)),
        "manifest_sha256": sha256(target),
        "manifest_bytes": target.stat().st_size,
        "source_file_count": len(source_files),
        "run_file_count": len(run_files),
        "run_total_bytes": run_total_bytes,
        "excluded_self_referential": EXCLUSIONS,
        "protected_added": added,
        "protected_changed": changed,
        "protected_removed": [],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()