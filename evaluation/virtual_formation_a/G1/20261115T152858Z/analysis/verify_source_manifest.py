"""Independently verify the G1 rework source-and-evidence manifest.

This verifier rehashes every file listed in the manifest, every source file,
and every protected-snapshot entry from disk.  It also rechecks the G0
whitelist, the reported budgets and quadrature comparisons, setup registration,
git state, and preservation of both old G1 evidence trees.  It writes nothing
except its console log.
"""
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
RUN_ID = RUN.name
MANIFEST_PATH = RUN / "source_manifest.json"
MANIFEST = json.loads(MANIFEST_PATH.read_text())
SCHEMA = "virtual_rigid_a.g1.rework.source_manifest.v2"
STAGE = "G1_REWORK_01"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
REWORK_DOC = "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md"
OLD_SUBMITTED = (REPO_ROOT / "evaluation" / "virtual_formation_a" /
                 "G1" / "20261115T150639Z")
OLD_PRELIMINARY = (REPO_ROOT / "evaluation" / "virtual_formation_a" /
                   "G1" / "20261115T150114Z")
G0_PATH = (REPO_ROOT / "evaluation" / "virtual_formation_a" / "G0" /
           "20261115T143321Z_rework" / "protected_before.json")
EXCLUSIONS = {
    "logs/source_manifest_verification.log",
    "logs/source_manifest_generation.log",
    "source_manifest.json",
}
ALLOWED_NEW = {
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/__init__.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/bounds.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/contracts.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/geometry.py",
    "tests/test_virtual_a_geometry.py",
    "tests/test_virtual_a_profile.py",
    "tools/virtual_a_benchmark.py",
}
ALLOWED_TRACKED = {"catkin_ws/src/fwis_v3_optimizer/setup.py"}
EXPECTED_OLD = {
    "submitted": {
        "manifest_sha256":
            "c40d3ac534aac957cc2206979ab85ef26f073bc6ca89fe5baf882d7cf124bc14",
        "tree_sha256":
            "d14cc53ee8026b250069547f3d441e1eac2f15b50c825f8f2112bfad65bf4739",
        "file_count": 38,
        "total_bytes": 3614233,
    },
    "preliminary": {
        "manifest_sha256":
            "55bf0586f414a469ddb0b434784adb2008239664d0a197deee61fbbae98f7ce4",
        "tree_sha256":
            "777edd42225a2feef23edfabab558aff980dc08e92dbe88bbbe85772eb685dc8",
        "file_count": 39,
        "total_bytes": 3597231,
    },
}
EXPECTED_ANALYTIC_BUDGET = 7.737011020460756e-08
EXPECTED_C2_BUDGET = 9.779693223310183e-09


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def tree_hash(root):
    root = Path(root)
    digest = hashlib.sha256()
    files = sorted(path for path in root.rglob("*") if path.is_file())
    for path in files:
        digest.update(path.relative_to(root).as_posix().encode("utf-8") + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
        digest.update(b"\n")
    return {
        "sha256": digest.hexdigest(),
        "file_count": len(files),
        "total_bytes": sum(path.stat().st_size for path in files),
    }


def canonical_protected_tree(snapshot):
    payload = {
        entry["path"]: {
            "type": entry["type"],
            "sha256": entry["sha256"],
            "size_bytes": entry["size_bytes"],
        }
        for entry in snapshot["protected_files"]
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def rehash_snapshot(snapshot):
    files = 0
    symlinks = 0
    errors = []
    for entry in snapshot["protected_files"]:
        path = REPO_ROOT / entry["path"]
        if entry["type"] == "symlink":
            symlinks += 1
            if not path.is_symlink():
                errors.append(entry["path"] + ": no longer a symlink")
                continue
            target = os.readlink(str(path))
            actual = hashlib.sha256(target.encode("utf-8")).hexdigest()
            if actual != entry["sha256"]:
                errors.append(entry["path"] + ": symlink target hash differs")
        else:
            files += 1
            if path.is_symlink() or not path.is_file():
                errors.append(entry["path"] + ": no longer a regular file")
                continue
            if path.stat().st_size != entry["size_bytes"]:
                errors.append(entry["path"] + ": size differs")
            if sha256(path) != entry["sha256"]:
                errors.append(entry["path"] + ": sha256 differs")
    return files, symlinks, errors


def git(*args, check=True):
    return subprocess.run(
        ["git"] + list(args), cwd=str(REPO_ROOT), check=check,
        capture_output=True, text=True)


class Checks:
    def __init__(self):
        self.results = []

    def run(self, name, function):
        try:
            detail = function()
        except Exception as exc:  # verification must report every failure
            self.results.append((name, False, "{}: {}".format(
                type(exc).__name__, exc)))
            print("[FAIL] {}: {}: {}".format(
                name, type(exc).__name__, exc))
        else:
            self.results.append((name, True, detail))
            print("[PASS] {}: {}".format(name, detail))


def main():
    print("G1 rework source manifest verification")
    print("run: {}".format(RUN.relative_to(REPO_ROOT)))
    print("manifest: {}".format(MANIFEST_PATH.relative_to(REPO_ROOT)))
    print("manifest_sha256: {}".format(sha256(MANIFEST_PATH)))
    print("verifier: {}".format(Path(__file__).resolve()))
    print("cwd: {}".format(Path.cwd()))
    print("verified_at_utc: {}".format(datetime.datetime.now(
        datetime.timezone.utc).isoformat()))
    print("excluded_self_referential: {}".format(
        json.dumps(sorted(EXCLUSIONS))))
    checks = Checks()

    def schema_and_scope():
        assert MANIFEST["schema"] == SCHEMA, MANIFEST["schema"]
        assert MANIFEST["stage"] == STAGE, MANIFEST["stage"]
        assert MANIFEST["run_id"] == RUN_ID, MANIFEST["run_id"]
        assert MANIFEST["baseline_head"] == BASELINE_HEAD
        assert MANIFEST["repo_root"] == str(REPO_ROOT)
        assert set(MANIFEST["excluded_self_referential"]) == EXCLUSIONS
        assert len(MANIFEST["excluded_self_referential"]) == len(EXCLUSIONS)
        assert (REPO_ROOT / REWORK_DOC).is_file()
        assert REWORK_DOC in MANIFEST["source_files"]
        return "schema, run id, baseline, exclusions and rework document"

    def run_file_coverage_and_hashes():
        actual = {}
        for path in sorted(RUN.rglob("*")):
            if path.is_file():
                relative = path.relative_to(RUN).as_posix()
                if relative not in EXCLUSIONS:
                    actual[relative] = {
                        "bytes": path.stat().st_size,
                        "sha256": sha256(path),
                    }
        listed = MANIFEST["run_files"]
        assert set(actual) == set(listed), {
            "extra": sorted(set(actual) - set(listed)),
            "missing": sorted(set(listed) - set(actual)),
        }
        for relative, record in listed.items():
            assert actual[relative] == record, relative
        total_bytes = sum(record["bytes"] for record in listed.values())
        assert MANIFEST["run_file_count"] == len(listed)
        assert MANIFEST["run_total_bytes"] == total_bytes
        for relative in EXCLUSIONS:
            assert (RUN / relative).is_file(), relative
        return "{} files, {} bytes; three exclusions present".format(
            len(listed), total_bytes)

    def source_file_hashes():
        old = json.loads((OLD_SUBMITTED / "source_manifest.json").read_text())
        expected = sorted(old["source_files"])
        expected.append(REWORK_DOC)
        expected = sorted(set(expected))
        assert len(expected) == 15, len(expected)
        listed = MANIFEST["source_files"]
        assert set(listed) == set(expected), {
            "extra": sorted(set(listed) - set(expected)),
            "missing": sorted(set(expected) - set(listed)),
        }
        assert MANIFEST["source_file_count"] == 15
        for relative, record in listed.items():
            path = REPO_ROOT / relative
            assert path.is_file(), relative
            assert path.stat().st_size == record["bytes"], relative
            assert sha256(path) == record["sha256"], relative
        return "15 source files rehashed from disk"

    def results_and_budgets():
        results = json.loads((RUN / "results.json").read_text())
        evidence = json.loads((RUN / "analysis" / "budget_evidence.json").read_text())
        benchmark = json.loads((RUN / "benchmark" / "summary.json").read_text())
        assert results["status"] == "PASS_PENDING_CODEX_RECHECK"
        assert results["codex_acceptance"] == "NOT_RUN_NOT_CLAIMED"
        assert results["scope"]["g2_entered"] is False
        assert results["scope"]["rework_document_sha256"] == sha256(
            REPO_ROOT / REWORK_DOC)
        assert results["tests"]["geometry"]["count"] == 20
        assert results["tests"]["profile"]["count"] == 22
        assert results["tests"]["benchmark"]["checks"] == 26
        assert len(benchmark["checks"]) == 26
        assert all(check.get("pass") for check in benchmark["checks"])
        assert benchmark["status"] == "PASS"
        analytic = evidence["analytic_double_lane_change"]
        c2 = evidence["c2_nonzero_g"]
        assert analytic["budget"]["position_error_bound_m"] == \
            EXPECTED_ANALYTIC_BUDGET
        assert c2["budget"]["position_error_bound_m"] == \
            EXPECTED_C2_BUDGET
        assert EXPECTED_ANALYTIC_BUDGET <= 1e-7
        assert EXPECTED_C2_BUDGET <= 1e-7
        assert analytic["budget"]["angle_error_bound_rad"] <= 1e-9
        assert c2["budget"]["angle_error_bound_rad"] <= 1e-9
        assert analytic["raw_recompute"]["reported_total_leq_1e_7"] is True
        assert analytic["raw_recompute"]["reported_total_equals_recompute"] \
            is True
        assert c2["raw_recompute"]["reported_total_leq_1e_7"] is True
        assert c2["raw_recompute"]["reported_total_equals_recompute"] is True
        assert analytic["benchmark_audit"]["fields_consistent"] is True
        assert analytic["benchmark_audit"]["totals_consistent"] is True
        assert c2["benchmark_audit"]["fields_consistent"] is True
        assert c2["benchmark_audit"]["totals_consistent"] is True
        assert analytic["representative_partial_queries"][
            "all_within_bound"] is True
        assert c2["all_within_reported_budget"] is True

        def within(row):
            return row["position_error_m"] <= EXPECTED_C2_BUDGET

        assert all(within(row) for row in c2["stations"])
        for row in analytic["representative_partial_queries"]["stations"]:
            assert row["quadrature_error_m"] <= EXPECTED_ANALYTIC_BUDGET
            assert row["local_panel_count"] <= 1
            assert row["local_panel_width_m"] <= \
                row["max_local_panel_width_m"] + 1e-15

        hand = evidence["hand_parameter_checks"]["cases"]
        m4 = evidence["hand_parameter_checks"]["m4"]
        for row in hand:
            n = row["subintervals_n"]
            h = row["h_m"]
            assert n % 2 == 0 and n >= 2
            width = h / n * 2.0
            per_panel = width ** 5 * m4 / 2880.0
            assert math.isclose(
                row["expected_full_remainder_m"],
                (n // 2) * per_panel, rel_tol=1e-13)
            assert math.isclose(
                row["expected_local_remainder_m"], per_panel, rel_tol=1e-13)
            assert row["old_formula_is_underestimate"] is True
            assert row["segment_remainder_m"] > row[
                "old_n_plus_one_d_budget_m"]

        analytic_budget = analytic["budget"]
        width = analytic_budget["panel_widths_m"][0]
        per_panel = width ** 5 * analytic_budget["m4_per_segment"][0] / 2880.0
        recomputed_per_coordinate = (
            (analytic_budget["panels_per_segment"][0] // 2)
            * per_panel + per_panel)
        recomputed_per_coordinate *= 2.0
        assert math.isclose(
            analytic_budget["simpson_per_coordinate_m"],
            recomputed_per_coordinate, rel_tol=2e-14)
        recomputed_guard = 1e-10 * (
            analytic_budget["total_full_simpson_panels"]
            + analytic_budget["total_local_simpson_panels"] + 8) + (
                12.0 * analytic_budget["angle_error_bound_rad"])
        assert math.isclose(
            analytic_budget["floating_point_guard_m"], recomputed_guard,
            rel_tol=1e-12)

        c2_budget = c2["budget"]
        c2_per_coordinate = 0.0
        for index, width in enumerate(c2_budget["panel_widths_m"]):
            per_panel = width ** 5 * c2_budget[
                "m4_per_segment"][index] / 2880.0
            c2_per_coordinate += (
                c2_budget["panels_per_segment"][index] // 2) * per_panel + \
                per_panel
        assert math.isclose(
            c2_budget["simpson_per_coordinate_m"], c2_per_coordinate,
            rel_tol=2e-14)

        refinement = results["refinement"]
        assert refinement["analytic_initial_panels_per_segment"] == [300, 300]
        assert refinement["analytic_final_panels_per_segment"] == [600, 600]
        assert refinement["analytic_refinement_steps"] == 2
        assert refinement["analytic_total_complete_panels"] == 600
        assert refinement["analytic_total_local_panels"] == 2
        assert refinement["c2_initial_panels_per_segment"] == [50, 50]
        assert refinement["c2_final_panels_per_segment"] == [50, 50]
        assert refinement["c2_refinement_steps"] == 0
        assert refinement["c2_total_complete_panels"] == 50
        assert refinement["c2_total_local_panels"] == 2
        assert results["tests"]["baseline_geometry_regression"][
            "unmodified_runner_attempt"]["status"] == "FAILED_BEFORE_TESTS"
        assert results["tests"]["baseline_geometry_regression"][
            "documented_wrapper_run"]["status"] == "PASS"
        assert results["constraints"]["codex_acceptance"] == \
            "NOT_RUN_NOT_CLAIMED"
        assert results["constraints"]["continuous_domain_proof"] == \
            "NOT_CLAIMED_SAMPLED_EVIDENCE_ONLY"
        return (
            "20 geometry + 22 profile tests; 26/26 benchmark checks; "
            "analytic {:.3e} m and C2 {:.3e} m within 1e-7 m; "
            "sampled C2/partial errors within budgets").format(
                EXPECTED_ANALYTIC_BUDGET, EXPECTED_C2_BUDGET)

    def protected_snapshots_current_and_unchanged():
        before = json.loads((RUN / "legacy" / "protected_before.json").read_text())
        after = json.loads((RUN / "legacy" / "protected_after.json").read_text())
        assert before["protected_tree_sha256"] == canonical_protected_tree(before)
        assert after["protected_tree_sha256"] == canonical_protected_tree(after)
        assert before["protected_tree_sha256"] == after[
            "protected_tree_sha256"]
        assert before["protected_file_count"] == after["protected_file_count"]
        files, symlinks, errors = rehash_snapshot(before)
        assert not errors, errors[:10]
        files_after, symlinks_after, errors_after = rehash_snapshot(after)
        assert not errors_after, errors_after[:10]
        assert (files, symlinks) == (files_after, symlinks_after)
        manifest_protected = MANIFEST["protected"]
        assert manifest_protected["before_tree_sha256"] == \
            before["protected_tree_sha256"]
        assert manifest_protected["after_tree_sha256"] == \
            after["protected_tree_sha256"]
        assert manifest_protected["before_file_count"] == \
            before["protected_file_count"]
        assert manifest_protected["after_file_count"] == \
            after["protected_file_count"]
        assert sha256(RUN / "legacy" / "protected_before.json") == \
            manifest_protected["before_sha256"]
        assert sha256(RUN / "legacy" / "protected_after.json") == \
            manifest_protected["after_sha256"]
        assert manifest_protected["unchanged_within_run"] is True
        assert manifest_protected["snapshot_files_byte_identical"] is False
        return (
            "{} files + {} symlinks rehashed before and after; tree {} "
            "unchanged; snapshots differ only by generated_at").format(
                files, symlinks, before["protected_tree_sha256"])

    def g0_comparison_and_whitelist():
        g0 = json.loads(G0_PATH.read_text())
        assert canonical_protected_tree(g0) == g0["protected_tree_sha256"]
        assert g0["protected_tree_sha256"] == \
            "ff58f2b5c6acfbc207327202f875e5e9a20734ef09ebb6070ee07edce6ebf091"
        assert g0["protected_file_count"] == 3120
        assert MANIFEST["protected"]["g0_sha256"] == sha256(G0_PATH)
        assert MANIFEST["protected"]["g0_tree_sha256"] == \
            g0["protected_tree_sha256"]
        comparison_path = RUN / "protection_diff_vs_g0.json"
        assert sha256(comparison_path) == \
            MANIFEST["protected"]["comparison_sha256"]
        comparison = json.loads(comparison_path.read_text())
        added = {row["path"] for row in comparison["added_vs_g0"]}
        changed = {row["path"] for row in comparison["changed_vs_g0"]}
        assert added == ALLOWED_NEW, sorted(added ^ ALLOWED_NEW)
        assert changed == ALLOWED_TRACKED, sorted(changed ^ ALLOWED_TRACKED)
        assert comparison["removed_vs_g0"] == []
        assert comparison["authorization"]["only_authorized_changes"] is True
        assert comparison["authorization"][
            "additions_exactly_whitelisted"] is True
        assert comparison["authorization"][
            "tracked_changes_exactly_whitelisted"] is True
        assert comparison["protected_trees_unchanged_within_run"] is True
        assert comparison["g0_snapshot"]["protected_tree_sha256"] == \
            g0["protected_tree_sha256"]
        assert MANIFEST["protected"]["added_vs_g0"] == sorted(ALLOWED_NEW)
        assert MANIFEST["protected"]["changed_vs_g0"] == sorted(ALLOWED_TRACKED)
        return (
            "seven authorized additions; setup.py sole tracked change; "
            "no removals; G0 tree verified")

    def setup_registration():
        log = (RUN / "logs" / "setup_registration.log").read_text()
        assert "fwis_v3_optimizer.formation_continuous_a" in log
        assert log.count("exit_code=0") == 3
        setup_path = REPO_ROOT / "catkin_ws/src/fwis_v3_optimizer/setup.py"
        setup_text = setup_path.read_text()
        assert "formation_continuous_a" in setup_text
        stat_line = git(
            "diff", "--stat", "--",
            "catkin_ws/src/fwis_v3_optimizer/setup.py").stdout.strip()
        assert stat_line.endswith("1 file changed, 4 insertions(+), 1 deletion(-)"), \
            stat_line
        protection = json.loads(
            (RUN / "protection_diff_vs_g0.json").read_text())
        assert protection["setup_diff"]["insertions"] == 4
        assert protection["setup_diff"]["deletions"] == 1
        assert MANIFEST["protected"]["setup_diff_stat"] == stat_line
        assert MANIFEST["source_files"][
            "catkin_ws/src/fwis_v3_optimizer/setup.py"]["sha256"] == \
            "de6138b17b91160b6827d9e7f2b89925bc9cc3f4705f1800387492708025eee8"
        return "both packages registered; setup.py diff is 4 insertions / 1 deletion"

    def git_state():
        head = git("rev-parse", "HEAD").stdout.strip()
        branch = git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
        status = git("status", "--short").stdout.splitlines()
        staged = git("diff", "--cached", "--binary").stdout
        staged_stat = git("diff", "--cached", "--stat").stdout.strip()
        ancestor = git(
            "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD",
            check=False).returncode
        assert head == BASELINE_HEAD
        assert branch == "feature/virtual-rigid-formation-a"
        assert ancestor == 0
        assert status == MANIFEST["git"]["status_short"], {
            "manifest": MANIFEST["git"]["status_short"],
            "actual": status,
        }
        assert staged == ""
        assert staged_stat == ""
        assert MANIFEST["git"]["head"] == head
        assert MANIFEST["git"]["branch"] == branch
        assert MANIFEST["git"]["committed"] == "NO"
        assert MANIFEST["git"]["pushed"] == "NO"
        assert MANIFEST["git"]["staged_diff_sha256"] == hashlib.sha256(
            b"").hexdigest()
        assert MANIFEST["git"]["staged_diff_stat"] == ""
        return (
            "HEAD is baseline 567bdecba967 on feature/virtual-rigid-formation-a; "
            "nothing staged; status matches manifest")

    def old_evidence_preservation():
        submitted = tree_hash(OLD_SUBMITTED)
        preliminary = tree_hash(OLD_PRELIMINARY)
        assert submitted["sha256"] == EXPECTED_OLD["submitted"]["tree_sha256"]
        assert submitted["file_count"] == EXPECTED_OLD["submitted"]["file_count"]
        assert submitted["total_bytes"] == EXPECTED_OLD["submitted"]["total_bytes"]
        assert preliminary["sha256"] == \
            EXPECTED_OLD["preliminary"]["tree_sha256"]
        assert preliminary["file_count"] == \
            EXPECTED_OLD["preliminary"]["file_count"]
        assert preliminary["total_bytes"] == \
            EXPECTED_OLD["preliminary"]["total_bytes"]
        assert sha256(OLD_SUBMITTED / "source_manifest.json") == \
            EXPECTED_OLD["submitted"]["manifest_sha256"]
        assert sha256(OLD_PRELIMINARY / "source_manifest.json") == \
            EXPECTED_OLD["preliminary"]["manifest_sha256"]
        previous = MANIFEST["previous_evidence"]
        assert previous["submitted_run"]["tree"] == submitted
        assert previous["preliminary_run"]["tree"] == preliminary
        assert previous["submitted_run"]["source_manifest_sha256"] == \
            EXPECTED_OLD["submitted"]["manifest_sha256"]
        assert previous["preliminary_run"]["source_manifest_sha256"] == \
            EXPECTED_OLD["preliminary"]["manifest_sha256"]
        results = json.loads((RUN / "results.json").read_text())
        assert results["previous_evidence"]["submitted_run"]["tree"][
            "sha256"] == submitted["sha256"]
        assert results["previous_evidence"]["preliminary_run"]["tree"][
            "sha256"] == preliminary["sha256"]
        assert all(result["status"] == "PRESERVED_NOT_MODIFIED"
                   for result in (results["previous_evidence"]["submitted_run"],
                                  results["previous_evidence"]["preliminary_run"]))
        changed = previous["changed_sources_vs_old"]
        assert set(changed) == {
            "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
            "formation_continuous_a/bounds.py",
            "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
            "formation_continuous_a/geometry.py",
            "tests/test_virtual_a_profile.py",
            "tools/virtual_a_benchmark.py",
        }
        for relative, record in changed.items():
            assert record["changed"] is True
            assert record["old_sha256"] != record["new_sha256"]
            assert record["new_sha256"] == MANIFEST["source_files"][relative][
                "sha256"]
        return (
            "old submitted tree {} (38 files) and preliminary tree {} "
            "(39 files) unchanged; old manifests unchanged").format(
                submitted["sha256"], preliminary["sha256"])

    def run_id_and_evidence_presence():
        text = (RUN / "logs" / "run_id.txt").read_text().strip()
        assert text == "VA_RUN=evaluation/virtual_formation_a/G1/" + RUN_ID
        for relative in (
                "CLAUDE_REPORT.md", "COMMANDS.md", "environment.json",
                "results.json", "protection_diff_vs_g0.json",
                "analysis/budget_evidence.json",
                "analysis/write_source_manifest.py",
                "analysis/verify_source_manifest.py",
                "benchmark/summary.json", "logs/profile.log",
                "logs/geometry.log", "logs/benchmark.log",
                "logs/legacy_geometry_unmodified_gate.log",
                "logs/legacy_geometry.log"):
            assert (RUN / relative).is_file(), relative
            assert relative in MANIFEST["run_files"], relative
        return "run id and all primary evidence files are present and hashed"

    checks.run("schema_and_scope", schema_and_scope)
    checks.run("run_file_coverage_and_hashes", run_file_coverage_and_hashes)
    checks.run("source_file_hashes", source_file_hashes)
    checks.run("results_and_budgets", results_and_budgets)
    checks.run(
        "protected_snapshots_current_and_unchanged",
        protected_snapshots_current_and_unchanged)
    checks.run("g0_comparison_and_g1_whitelist",
               g0_comparison_and_whitelist)
    checks.run("setup_registration", setup_registration)
    checks.run("git_state", git_state)
    checks.run("old_evidence_preservation", old_evidence_preservation)
    checks.run("run_id_and_evidence_presence", run_id_and_evidence_presence)

    failures = [row for row in checks.results if not row[1]]
    print("checked_checks: {}".format(len(checks.results)))
    print("failed_checks: {}".format(len(failures)))
    print("status: {}".format("PASS" if not failures else "FAIL"))
    print("exit_code: {}".format(0 if not failures else 1))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())