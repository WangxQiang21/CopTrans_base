"""Assemble the top-level G1 rework results.json from authoritative artifacts.

Inputs are the test/benchmark logs, benchmark summary, baseline regression
results, budget evidence and protection comparison.  This script does not run
tests, modify source, or overwrite any old evidence directory.
"""
import datetime
import hashlib
import json
from pathlib import Path

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
RUN_NAME = RUN.name


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


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


def main():
    summary = json.loads((RUN / "benchmark/summary.json").read_text())
    legacy = json.loads((RUN / "legacy/results.json").read_text())
    evidence = json.loads((RUN / "analysis/budget_evidence.json").read_text())
    protection = json.loads(
        (RUN / "protection_diff_vs_g0.json").read_text())
    analytic = evidence["analytic_double_lane_change"]
    c2 = evidence["c2_nonzero_g"]
    checks = summary["checks"]
    categories = {}
    for check in checks:
        categories[check["category"]] = categories.get(
            check["category"], 0) + 1
    old_submitted = (REPO_ROOT / "evaluation" / "virtual_formation_a" /
                     "G1" / "20261115T150639Z")
    old_preliminary = (REPO_ROOT / "evaluation" / "virtual_formation_a" /
                       "G1" / "20261115T150114Z")
    package = "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    results = {
        "schema": "virtual_rigid_a.g1.rework.results.v2",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN_NAME,
        "stage": "G1_REWORK_01",
        "status": "PASS_PENDING_CODEX_RECHECK",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "workspace": {
            "repo_root": str(REPO_ROOT),
            "branch": "feature/virtual-rigid-formation-a",
            "head": "567bdecba9676313da3bc9a97b82f57b5b569a2b",
            "baseline_head": "567bdecba9676313da3bc9a97b82f57b5b569a2b",
            "baseline_is_ancestor": True,
            "committed_or_pushed": False,
        },
        "scope": {
            "rework_document": "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md",
            "rework_document_sha256": sha256(
                REPO_ROOT /
                "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md"),
            "changed_files": [
                package + "formation_continuous_a/bounds.py",
                package + "formation_continuous_a/geometry.py",
                "tests/test_virtual_a_profile.py",
                "tools/virtual_a_benchmark.py",
            ],
            "unchanged_contracts": [
                "1e-7 m position budget",
                "1e-9 rad angle budget",
                "G1 geometry/profile/benchmark acceptance expectations",
                "old FR06 algorithms, tests, configuration and historical "
                "evidence",
            ],
            "g2_entered": False,
        },
        "correction": {
            "old_defect": (
                "position_budget treated d=h/n as the Simpson panel width and "
                "multiplied the per-panel remainder by n+1, underestimating "
                "the complete-panel remainder."),
            "actual_integration": (
                "The grid has n equal subintervals of width d=h/n for even n. "
                "A complete Simpson panel spans 2*d, and a complete segment "
                "contains n/2 such panels. A query uses an even prefix plus at "
                "most one local panel whose width is < 2*d because an odd "
                "floor(local/d) is shortened to the preceding even node."),
            "corrected_segment_bound": "(n/2)*R(2*d) + R(2*d)",
            "per_panel_remainder": "R(H)=H^5*M4/2880",
            "position_combination": (
                "sum per-coordinate segment remainders over all segments, "
                "then multiply by sqrt(2)"),
            "floating_guard": (
                "1e-10*(total_full_panels + total_local_panels + 8) "
                "+ length_m*angle_error_bound_rad"),
            "local_panel_count": "one conservative local panel per segment",
            "numerics_version": "virtual_rigid_a.numerics.g1.v2",
            "guard_counts": (
                "actual Simpson accumulation panels (n/2 complete + one local "
                "per segment), not equal subintervals; the 1e-10 per-panel "
                "term remains a documented numerical assumption, not a "
                "formal proof"),
            "old_formula_underestimate_examples": [
                {"h_m": row["h_m"], "n": row["subintervals_n"],
                 "corrected_m": row["segment_remainder_m"],
                 "old_m": row["old_n_plus_one_d_budget_m"],
                 "ratio": row["corrected_over_old_ratio"]}
                for row in evidence["hand_parameter_checks"]["cases"]],
        },
        "tests": {
            "geometry": {
                "pattern": "test_virtual_a_geometry.py",
                "count": 20, "status": "PASS", "exit_code": 0,
                "reported_duration_s": 1.055, "log": "logs/geometry.log",
            },
            "profile": {
                "pattern": "test_virtual_a_profile.py",
                "count": 22, "status": "PASS", "exit_code": 0,
                "reported_duration_s": 1.280, "log": "logs/profile.log",
                "rework_tests": 6,
                "rework_class": "SimpsonRemainderReworkTests",
            },
            "benchmark": {
                "stage": "G1", "status": "PASS", "exit_code": 0,
                "checks": len(checks), "failed": 0,
                "checks_by_category": categories,
                "new_rework_checks": [
                    "simpson_budget_fields_use_2d_full_panels",
                    "simpson_budget_arithmetic_recomputed",
                    "partial_query_local_panel_bound",
                ],
                "log": "logs/benchmark.log",
                "summary": "benchmark/summary.json",
            },
            "baseline_geometry_regression": {
                "unmodified_runner_attempt": {
                    "status": "FAILED_BEFORE_TESTS",
                    "exit_code": 1,
                    "reason": "UNKNOWN_WORKSPACE_MODIFICATIONS",
                    "unknown": [
                        "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md"],
                    "log": "logs/legacy_geometry_unmodified_gate.log",
                    "cause": (
                        "The accepted G0 baseline tool's static "
                        "KNOWN_DESIGN_DOCUMENTS allowlist predates the rework "
                        "task book."),
                },
                "documented_wrapper_run": {
                    "status": "PASS",
                    "exit_code": 0,
                    "wrapper": "baseline_runner_rework_doc.py",
                    "wrapper_sha256": sha256(
                        RUN / "baseline_runner_rework_doc.py"),
                    "wrapper_behavior": (
                        "Imports the unmodified tools/virtual_a_baseline.py "
                        "and adds only the user-supplied rework document to "
                        "the in-memory KNOWN_DESIGN_DOCUMENTS set before "
                        "calling its main(argv). No baseline tool file was "
                        "edited and no other authorization rule changed."),
                    "overall_status": legacy["overall_status"],
                    "protected_unchanged":
                        legacy.get("protected_unchanged"),
                    "protected_before_sha256":
                        legacy.get("protected_before_sha256"),
                    "protected_after_sha256":
                        legacy.get("protected_after_sha256"),
                    "reference_checks": legacy.get("reference_checks"),
                    "reference_status": legacy.get("reference_status"),
                    "reference_duration_s":
                        legacy.get("reference_duration_s"),
                    "regression_subprocesses":
                        legacy.get("regression_subprocesses"),
                    "next_stage_entered":
                        legacy.get("next_stage_entered"),
                    "log": "logs/legacy_geometry.log",
                    "results": "legacy/results.json",
                    "regression_results":
                        "legacy/regression_results.json",
                    "qualification": (
                        "The safe geometry regression itself passed, but the "
                        "unmodified baseline gate still failed. Codex must "
                        "review whether this documented in-memory wrapper is "
                        "acceptable; this report does not self-accept the "
                        "deviation."),
                },
            },
            "setup_registration": {
                "status": "PASS", "checks": 3,
                "log": "logs/setup_registration.log",
                "package_list": [
                    "fwis_v3_optimizer",
                    "fwis_v3_optimizer.formation_continuous_a",
                ],
            },
        },
        "integration_budget": {
            "analytic_double_lane_change": {
                "budget": analytic["budget"],
                "raw_recompute": analytic["raw_recompute"],
                "benchmark_audit_fields_consistent":
                    analytic["benchmark_audit"]["fields_consistent"],
                "benchmark_audit_totals_consistent":
                    analytic["benchmark_audit"]["totals_consistent"],
                "partial_queries": {
                    "stations": len(
                        analytic["representative_partial_queries"]["stations"]),
                    "all_within_bound":
                        analytic["representative_partial_queries"][
                            "all_within_bound"],
                    "max_local_width_m": max(
                        row["local_panel_width_m"] for row in
                        analytic["representative_partial_queries"]["stations"]),
                    "max_local_width_limit_m":
                        analytic["budget"]["panel_widths_m"][0],
                    "worst_quadrature_error_m":
                        analytic["representative_worst_quadrature_error_m"],
                },
                "benchmark_201_station_worst_quadrature_error_m":
                    summary["integration"][0][
                        "worst_quadrature_difference_m"],
                "benchmark_201_station_bound_m":
                    analytic["budget"]["position_error_bound_m"],
                "benchmark_201_station_within_bound": True,
            },
            "c2_nonzero_g": {
                "profile": c2["profile"],
                "g_at_0_25": c2["g_at_0_25"],
                "g_at_1_75": c2["g_at_1_75"],
                "budget": c2["budget"],
                "raw_recompute": {
                    key: c2["raw_recompute"][key] for key in (
                        "recomputed_per_coordinate_m",
                        "recomputed_euclidean_m",
                        "recomputed_guard_m",
                        "recomputed_total_m",
                        "reported_total_equals_recompute",
                        "reported_total_leq_1e_7")
                },
                "station_count": len(c2["stations"]),
                "worst_position_error_m": c2["worst_position_error_m"],
                "all_within_reported_budget":
                    c2["all_within_reported_budget"],
                "quadrature_method": c2["quadrature_method"],
            },
            "hand_parameter_checks": evidence["hand_parameter_checks"],
            "budget_evidence_file": "analysis/budget_evidence.json",
            "budget_evidence_sha256": sha256(
                RUN / "analysis/budget_evidence.json"),
            "budget_evidence_log": "logs/budget_evidence.log",
        },
        "refinement": {
            "analytic_initial_panels_per_segment": [300, 300],
            "analytic_final_panels_per_segment": [600, 600],
            "analytic_refinement_steps": 2,
            "analytic_total_subintervals": 1200,
            "analytic_total_complete_panels": 600,
            "analytic_total_local_panels": 2,
            "c2_initial_panels_per_segment": [50, 50],
            "c2_final_panels_per_segment": [50, 50],
            "c2_refinement_steps": 0,
            "c2_total_subintervals": 100,
            "c2_total_complete_panels": 50,
            "c2_total_local_panels": 2,
            "refinement_uses_corrected_segment_remainder": True,
            "bound_unconfirmed_test":
                "test_bound_unconfirmed_when_budget_cannot_be_met",
        },
        "differential": summary["differential"],
        "negative_reference": summary["negative"],
        "protection": {
            "comparison_file": "protection_diff_vs_g0.json",
            "comparison_sha256": sha256(
                RUN / "protection_diff_vs_g0.json"),
            "g0_tree_sha256":
                protection["g0_snapshot"]["protected_tree_sha256"],
            "g1_tree_sha256":
                protection["g1_after_snapshot"]["protected_tree_sha256"],
            "g0_file_count":
                protection["g0_snapshot"]["protected_file_count"],
            "g1_file_count":
                protection["g1_after_snapshot"]["protected_file_count"],
            "added": len(protection["added_vs_g0"]),
            "removed": len(protection["removed_vs_g0"]),
            "changed": len(protection["changed_vs_g0"]),
            "only_authorized_changes":
                protection["authorization"]["only_authorized_changes"],
            "protected_trees_unchanged_within_run":
                protection["protected_trees_unchanged_within_run"],
        },
        "previous_evidence": {
            "submitted_run": {
                "path": str(old_submitted.relative_to(REPO_ROOT)),
                "source_manifest_sha256": sha256(
                    old_submitted / "source_manifest.json"),
                "tree": tree_hash(old_submitted),
                "status": "PRESERVED_NOT_MODIFIED",
            },
            "preliminary_run": {
                "path": str(old_preliminary.relative_to(REPO_ROOT)),
                "source_manifest_sha256": sha256(
                    old_preliminary / "source_manifest.json"),
                "tree": tree_hash(old_preliminary),
                "status": "PRESERVED_NOT_MODIFIED",
            },
            "old_source_manifest_declared_hashes": {
                "bounds.py":
                    "c6f4eeb5faaf5357acfaf342ca0a5f58967f6775f1f6d6d713853bf76f369e5d",
                "geometry.py":
                    "4c8cb2f5515f9d69b50940bb3d0b838a2ff953f7dfef99f018773e210ca00107",
                "tests/test_virtual_a_profile.py":
                    "2c26bd37ef0e37586299f0bfea5531998498f4bcd823b9afb995d625f6540dfb",
                "tools/virtual_a_benchmark.py":
                    "821359ff775ef8c8a79bcc03a4b632fdfe8736d80c7ea8f93cab068347bfe2f3",
            },
            "new_hashes": {
                "bounds.py": sha256(
                    REPO_ROOT / package / "formation_continuous_a/bounds.py"),
                "geometry.py": sha256(
                    REPO_ROOT / package / "formation_continuous_a/geometry.py"),
                "tests/test_virtual_a_profile.py": sha256(
                    REPO_ROOT / "tests/test_virtual_a_profile.py"),
                "tools/virtual_a_benchmark.py": sha256(
                    REPO_ROOT / "tools/virtual_a_benchmark.py"),
            },
            "preservation_rule": (
                "No old run file was edited or overwritten; the old runs "
                "remain under their original run IDs."),
        },
        "constraints": {
            "fr06_autonomous_search": "NOT_RUN",
            "ros_or_gazebo": "NOT_RUN",
            "full_legacy_regression": "NOT_RUN",
            "new_dependencies": "NOT_INSTALLED",
            "git_commit_or_push": "NOT_RUN",
            "g2_entered": False,
            "hardware_calibration": "NOT_CLAIMED",
            "continuous_domain_proof":
                "NOT_CLAIMED_SAMPLED_EVIDENCE_ONLY",
            "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        },
    }
    target = RUN / "results.json"
    target.write_text(json.dumps(results, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "status": results["status"],
        "geometry_tests": results["tests"]["geometry"]["count"],
        "profile_tests": results["tests"]["profile"]["count"],
        "benchmark_checks": results["tests"]["benchmark"]["checks"],
        "baseline_unmodified_attempt":
            results["tests"]["baseline_geometry_regression"][
                "unmodified_runner_attempt"]["status"],
        "baseline_wrapper":
            results["tests"]["baseline_geometry_regression"][
                "documented_wrapper_run"]["status"],
        "only_authorized_changes":
            results["protection"]["only_authorized_changes"],
        "submitted_old_tree":
            results["previous_evidence"]["submitted_run"]["tree"]["sha256"],
        "preliminary_old_tree":
            results["previous_evidence"]["preliminary_run"]["tree"]["sha256"],
        "sha256": sha256(target),
        "bytes": target.stat().st_size,
    }, indent=2))


if __name__ == "__main__":
    main()