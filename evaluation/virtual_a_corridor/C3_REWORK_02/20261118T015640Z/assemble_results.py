"""Assemble C3_REWORK_02 evidence from the recorded logs and replays.

Evidence harness only: reads JSON/log files produced by the validation run,
computes the eligibility arithmetic reported in SUMMARY.md, and writes
results.json.  It does not solve, publish or touch ROS.
"""
import hashlib
import json
import os
import re
import subprocess


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
PIPELINE = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/pipeline.py")
CORRIDOR = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/corridor.py")
CONTRACTS = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/contracts.py")
OPTIMIZER = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/optimizer.py")
RECOVERY_TEST = "tests/test_virtual_a_corridor_recovery.py"
PIPELINE_TEST = "tests/test_virtual_a_pipeline.py"


def sha256(path):
    digest = hashlib.sha256()
    with open(os.path.join(ROOT, path), "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load(rel):
    with open(os.path.join(HERE, rel), "r", encoding="utf-8") as handle:
        return json.load(handle)


def test_meta():
    entries = []
    with open(os.path.join(HERE, "test_logs", "run_meta.log"),
              "r", encoding="utf-8") as handle:
        for line in handle:
            parts = line.rstrip("\n").split("\t")
            if len(parts) != 5:
                continue
            command, log, start, end, code = parts
            log_path = log if os.path.isabs(log) else os.path.join(ROOT, log)
            with open(log_path, "r", encoding="utf-8") as logf:
                text = logf.read()
            ran = re.search(r"^Ran (\d+) tests? in ([\d.]+)s$", text, re.M)
            entries.append({
                "command": command,
                "log": log,
                "start": start.split("=", 1)[1],
                "end": end.split("=", 1)[1],
                "exit": int(code.split("=", 1)[1]),
                "tests_run": int(ran.group(1)) if ran else None,
                "duration_s": float(ran.group(2)) if ran else None,
                "ok": bool(re.search(r"^OK$", text, re.M)),
                "test_names": re.findall(r"^(test_\w+) \(", text, re.M),
            })
    return entries


def direct_adjacent_segments(allocation, failure_index):
    """Return the two allocation segments meeting at the adjacent switch."""
    for position in range(1, len(allocation)):
        left = allocation[position - 1]
        right = allocation[position]
        left_index = int(left[2])
        right_index = int(right[2])
        if right_index != left_index + 1:
            continue
        if failure_index not in (left_index, right_index):
            continue
        return {
            "switch_position": position,
            "switch_sigma": float(right[0]),
            "left_corridor": left_index,
            "right_corridor": right_index,
            "left_segment": [float(left[0]), float(left[1])],
            "right_segment": [float(right[0]), float(right[1])],
        }
    return None


def main():
    s1 = load("replays/S1_corridor_reallocation.json")
    s3 = load("replays/S3_legacy_success.json")
    exhausted = load("replays/initial_pool_exhaustion.json")
    tests = test_meta()

    reallocation = s1["reallocation"]
    failure_sigma = float(reallocation["failure_sigma"])
    adjacency = direct_adjacent_segments(
        reallocation["old_allocation"],
        int(reallocation["failure_corridor_index"]))
    in_left = (adjacency["left_segment"][0] <= failure_sigma
               <= adjacency["left_segment"][1])
    in_right = (adjacency["right_segment"][0] <= failure_sigma
                <= adjacency["right_segment"][1])
    failure_segment = next(
        [float(entry[0]), float(entry[1]), int(entry[2])]
        for entry in reallocation["old_allocation"]
        if float(entry[0]) <= failure_sigma <= float(entry[1]))
    was_switch_adjacent = bool(in_left or in_right)

    names = {name for entry in tests for name in entry["test_names"]}

    source_hashes = {
        rel: sha256(rel) for rel in (
            PIPELINE, CORRIDOR, OPTIMIZER, CONTRACTS,
            RECOVERY_TEST, PIPELINE_TEST)
    }
    rework_01_hashes = {
        PIPELINE: "9b33e1cabd11a23e40b511f4a1df71f6b52f3b89ccee0c0c98920dcfca02e4ec",
        CORRIDOR: "f08d7701dcb94428854f370bd4393484af3fcca4ab97df1208d955e2fc9e1564",
        OPTIMIZER: "1c49b426e8b5065f6e67847e80a35a01e296ec7099a10fd13472f4c4c3ed06c2",
        CONTRACTS: "bf46feceb404022f023f6b50a41928b22e23a5ace963071f53e0800b434f1a51",
        RECOVERY_TEST: "bd9c4f93de50518b5268992153bcbb707ae886fac38aef2f39354fbcbd1865db",
        PIPELINE_TEST: "e0e05632773e66d27ace9c4a933ffcbfda4c293a84ea4f0350f29531cbe6b079",
    }
    unchanged = {
        rel: source_hashes[rel] == rework_01_hashes[rel]
        for rel in rework_01_hashes
    }

    try:
        head = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        branch = subprocess.check_output(
            ["git", "branch", "--show-current"], cwd=ROOT, text=True).strip()
        porcelain = subprocess.check_output(
            ["git", "status", "--porcelain=v1"], cwd=ROOT, text=True)
    except (OSError, subprocess.CalledProcessError):
        head = branch = None
        porcelain = ""

    acceptance = {
        "s1_far_failure_no_recovery": (
            s1["recovery_count"] == 0 and s1["recoveries"] == []),
        "s1_failure_sigma_not_in_direct_adjacent_segments": (
            not was_switch_adjacent),
        "s1_reallocation_unavailable_and_not_consumed": (
            bool(reallocation["attempted"])
            and not bool(reallocation["available"])
            and not bool(reallocation["consumed_recovery"])
            and reallocation["new_allocation"] is None
            and reallocation["status"]
            == "fixed_sigma_reallocation_unavailable"),
        "s1_exactly_one_solve_round": (
            len(s1["rounds"]) == 1 and len(s1["attempts"]) == 1
            and len(s1["corridor_rounds"]) == 1),
        "s1_no_path_no_plan_not_executable": (
            not s1["executable"] and not s1["path_present"]
            and not s1["plan_present"] and s1["false_publications"] == 0),
        "positive_unit_switch_adjacent_reallocation": (
            "test_positive_candidate_moves_one_boundary_inside_true_overlap"
            in names),
        "negative_unit_far_failure_returns_none": (
            "test_failure_far_from_switch_or_degenerate_overlap_returns_none"
            in names),
        "pipeline_positive_one_recovery_then_stop": (
            "test_pipeline_reallocates_once_and_stops_on_second_failure"
            in names),
        "pipeline_negative_far_failure_spends_nothing": (
            "test_pipeline_far_from_switch_failure_does_not_spend_recovery"
            in names),
        "initial_pool_tests_preserved": all(name in names for name in (
            "test_route_audit_stops_before_spending_endpoint_family",
            "test_solver_auto_fallback_is_one_initial_and_no_fake_recovery",
            "test_numerical_failure_spends_one_endpoint_only_recovery")),
        "s3_legacy_success_preserved": (
            s3["status"] == "SUCCESS" and s3["executable"]
            and s3["path_present"] and s3["plan_present"]
            and s3["model_binding_ok"]
            and s3["geometry_audit_passed"]
            and s3["dynamic_audit_passed"]
            and s3["collision_audit_passed"]
            and s3["initial_count_total"] == 6
            and s3["duplicate_initial_launch_count"] == 0),
        "six_frozen_pipeline_scenes_preserved": (
            "test_six_frozen_scenes_pass_all_three_audits_autonomously"
            in names),
        "exhaustion_no_fake_recovery": (
            exhausted["recovery_count"] == 0
            and exhausted["recoveries"] == []
            and not exhausted["reallocation"]["attempted"]
            and not exhausted["path_present"]
            and not exhausted["plan_present"]),
        "solver_and_vehicle_limits_unmodified": (
            unchanged[OPTIMIZER] and unchanged[CONTRACTS]),
        "pipeline_test_unmodified": unchanged[PIPELINE_TEST],
        "all_targeted_tests_exit_zero": all(
            entry["exit"] == 0 and entry["ok"] for entry in tests),
        "head_unchanged_and_no_commit": (
            head == "9a19fd3e22d33188b13795f695757c3e4a36909f"),
        "false_publications_zero": (
            s1["false_publications"] == 0
            and s3["false_publications"] == 0
            and exhausted["false_publications"] == 0),
    }

    results = {
        "contract": "plan/virtual_a_corridor_v01/CLAUDE_C3_REWORK_02.md",
        "run_id": os.path.basename(HERE),
        "workspace": ROOT,
        "branch": branch,
        "head": head,
        "rework_01_baseline": (
            "reconstructed by applying C3_REWORK_01 source.diff to the "
            "/tmp/c3_rework_pre snapshot; all six hashes reproduced exactly"),
        "source_hashes": source_hashes,
        "rework_01_hashes": rework_01_hashes,
        "unchanged_from_rework_01": unchanged,
        "source_diff_scope": {
            "files": [CORRIDOR, PIPELINE, RECOVERY_TEST],
            "audit_thresholds_touched": False,
            "vehicle_limits_touched": False,
            "nlp_model_touched": False,
            "corridor_construction_or_allocation_algorithm_touched": False,
            "scope_note": (
                "corridor.py changes only reallocate_adjacent_portal_once; "
                "pipeline.py changes only the unavailable detail string; "
                "the recovery test file changes only tests."),
        },
        "tests": tests,
        "s1_replay": {
            "scene": "S1",
            "status": s1["status"],
            "reason": s1["reason"],
            "executable": s1["executable"],
            "path_present": s1["path_present"],
            "plan_present": s1["plan_present"],
            "route_count": s1["route_count"],
            "initial_count_total": s1["initial_count_total"],
            "initial_limit": s1["initial_limit"],
            "duplicate_initial_launch_count":
                s1["duplicate_initial_launch_count"],
            "recovery_count": s1["recovery_count"],
            "solver_attempts": len(s1["attempts"]),
            "geometry_rounds": len(s1["rounds"]),
            "corridor_rounds": len(s1["corridor_rounds"]),
            "failure_s_m": reallocation["failure_s_m"],
            "failure_sigma": failure_sigma,
            "failure_corridor_index":
                reallocation["failure_corridor_index"],
            "failure_point_residual_m":
                reallocation["failure_point_residual_m"],
            "first_failure_width_m": (
                s1["corridor_rounds"][0]["containment"]["first_failure"]
                ["width_m"]),
            "audit_min_width_m": (
                s1["corridor_rounds"][0]["containment"]["min_width_m"]),
            "failure_segment": failure_segment,
            "adjacent_switch": adjacency,
            "failure_in_left_adjacent_segment": in_left,
            "failure_in_right_adjacent_segment": in_right,
            "failure_is_switch_adjacent": was_switch_adjacent,
            "reallocation": reallocation,
            "recoveries": s1["recoveries"],
            "geometry_audit_passed": s1["geometry_audit_passed"],
            "dynamic_audit_passed": s1["dynamic_audit_passed"],
            "collision_audit_passed": s1["collision_audit_passed"],
            "false_publications": s1["false_publications"],
            "publication": s1["publication"],
            "ros": s1["ros"],
        },
        "s3_legacy_replay": {
            "status": s3["status"],
            "path_length_m": s3["path_length_m"],
            "executable": s3["executable"],
            "model_binding_ok": s3["model_binding_ok"],
            "route_count": s3["route_count"],
            "initial_count_total": s3["initial_count_total"],
            "recovery_count": s3["recovery_count"],
            "recoveries": s3["recoveries"],
            "geometry_audit_passed": s3["geometry_audit_passed"],
            "dynamic_audit_passed": s3["dynamic_audit_passed"],
            "collision_audit_passed": s3["collision_audit_passed"],
            "false_publications": s3["false_publications"],
        },
        "initial_pool_exhaustion_replay": {
            "status": exhausted["status"],
            "reason": exhausted["reason"],
            "initial_count_total": exhausted["initial_count_total"],
            "initial_limit": exhausted["initial_limit"],
            "recovery_count": exhausted["recovery_count"],
            "reallocation": exhausted["reallocation"],
            "path_present": exhausted["path_present"],
            "plan_present": exhausted["plan_present"],
            "false_publications": exhausted["false_publications"],
        },
        "original_evidence": {
            "c3_manifest_verification_exit": 0,
            "c3_manifest_self_sha256": sha256(
                "evaluation/virtual_a_corridor/C3/20261117T155212Z/"
                "manifest.sha256"),
            "c3_file_count": 28,
            "c3_tree_digest":
                "bb5558d4eda1083b07349d81f508eae483a982c283f25c9b2ae0214b2ee94640",
            "c3_rework_01_manifest_verification_exit": 0,
            "c3_rework_01_manifest_self_sha256": sha256(
                "evaluation/virtual_a_corridor/C3_REWORK_01/"
                "20261117T185418Z/manifest.sha256"),
            "c3_rework_01_file_count": 27,
            "c3_rework_01_tree_digest":
                "e2dee6995302cb52c5b51c0a20dd02442ad5103f9893cd4d467dfea86fd5c43d",
            "read_only": True,
        },
        "acceptance": acceptance,
        "all_acceptance_true": all(acceptance.values()),
        "not_run": ["ROS", "RViz", "publication", "C4", "C5"],
        "codex_status": "AWAITING_EXTERNAL_REVIEW",
    }
    with open(os.path.join(HERE, "results.json"), "w",
              encoding="utf-8") as handle:
        json.dump(results, handle, indent=1, sort_keys=True)
        handle.write("\n")
    print(json.dumps({
        "all_acceptance_true": results["all_acceptance_true"],
        "failed_checks": [
            key for key, value in acceptance.items() if not value],
        "s1_recovery_count": s1["recovery_count"],
        "s1_solver_attempts": len(s1["attempts"]),
        "s1_failure_is_switch_adjacent": was_switch_adjacent,
    }, indent=1, sort_keys=True))


if __name__ == "__main__":
    main()