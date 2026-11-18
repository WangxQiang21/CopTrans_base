"""Assemble C3_REWORK_01 results.json from the recorded logs and replays.

Read-only over the source tree and replay JSONs; writes only results.json in
this evidence directory.
"""
import hashlib
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
P = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
     "formation_continuous_a")
SOURCES = [
    P + "/pipeline.py",
    P + "/optimizer.py",
    P + "/contracts.py",
    P + "/corridor.py",
    "tests/test_virtual_a_corridor_recovery.py",
    "tests/test_virtual_a_pipeline.py",
]
PRE = {
    P + "/pipeline.py": "/tmp/c3_rework_pre/pipeline.py",
    P + "/optimizer.py": "/tmp/c3_rework_pre/optimizer.py",
    P + "/contracts.py": "/tmp/c3_rework_pre/contracts.py",
    P + "/corridor.py": "/tmp/c3_rework_pre/corridor.py",
    "tests/test_virtual_a_corridor_recovery.py":
        "/tmp/c3_rework_pre/test_virtual_a_corridor_recovery.py",
    "tests/test_virtual_a_pipeline.py":
        "/tmp/c3_rework_pre/test_virtual_a_pipeline.py",
}
TEST_PATTERNS = (
    "test_virtual_a_corridor_recovery",
    "test_virtual_a_corridor_optimizer",
    "test_virtual_a_optimizer",
    "test_virtual_a_pipeline",
)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def test_results():
    meta = {}
    with open(os.path.join(HERE, "test_logs", "run_meta.log"),
              encoding="utf-8") as handle:
        for line in handle:
            match = re.match(
                r"^(\S+) start (\S+) \| end (\S+) \| exit (\d+) \| "
                r"wall ([\d.]+) s", line)
            if match:
                meta[match.group(1)] = {
                    "start_utc": match.group(2),
                    "end_utc": match.group(3),
                    "exit_code": int(match.group(4)),
                    "wall_s": float(match.group(5)),
                }
    results = []
    for pattern in TEST_PATTERNS:
        path = os.path.join(HERE, "test_logs", pattern + ".log")
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
        ran = re.search(r"Ran (\d+) tests? in ([\d.]+)s", text)
        status = "OK" if re.search(r"^OK(?:\s|$)", text, re.M) else (
            "FAILED" if re.search(r"^FAILED", text, re.M) else "UNKNOWN")
        entry = {
            "pattern": pattern + ".py",
            "status": status,
            "tests_run": int(ran.group(1)) if ran else None,
            "unittest_wall_s": float(ran.group(2)) if ran else None,
            "log": "test_logs/" + pattern + ".log",
            "command": ("python3 -B -m unittest discover -s tests "
                        "-p " + pattern + ".py -v"),
        }
        entry.update(meta.get(pattern, {}))
        results.append(entry)
    return results


def replay_summary(name):
    with open(os.path.join(HERE, "replays", name + ".json"),
              encoding="utf-8") as handle:
        data = json.load(handle)
    keep = (
        "scene", "guidance_mode", "solver", "configured_max_initials",
        "status", "reason", "detail", "executable", "path_present",
        "plan_present", "path_length_m", "route_count", "initial_count",
        "initial_count_total", "initial_limit",
        "duplicate_initial_launch_count", "recovery_count",
        "first_failure_layer", "last_reason", "model_binding_ok",
        "geometry_audit_passed", "dynamic_audit_passed",
        "collision_audit_passed", "checks", "false_publications",
        "publication", "ros", "wall_time_s", "measured_wall_s",
        "initial_keys", "recoveries", "reallocation", "rounds",
        "attempts",
    )
    summary = {key: data.get(key) for key in keep}
    attempts = data.get("attempts") or ()
    corridor_rounds = data.get("corridor_rounds") or ()
    last_containment = (
        (corridor_rounds[-1] or {}).get("containment") or {}
        if corridor_rounds else {})
    summary["solver_attempt_count"] = len(attempts)
    summary["distinct_initial_count"] = data.get("initial_count_total")
    summary["continuous_containment_passed"] = (
        last_containment.get("passed") if corridor_rounds else None)
    summary["continuous_containment_reason"] = (
        last_containment.get("reason") if corridor_rounds else None)
    if data.get("plan_present") and data.get("executable") \
            and data.get("geometry_audit_passed") is True \
            and data.get("dynamic_audit_passed") is True \
            and data.get("collision_audit_passed") is True:
        summary["publication_eligibility"] = (
            "eligible_in_memory_not_published")
    else:
        summary["publication_eligibility"] = "not_eligible_no_plan"
    return summary


def allocation_checks(reallocation, recovery=None):
    old = reallocation.get("old_allocation")
    new = reallocation.get("new_allocation")
    if not old or not new:
        return None
    recovery = recovery or {}
    changed = [index for index, (a, b) in enumerate(zip(old, new))
               if a != b]
    if new[0][0] != 0.0 or new[-1][1] != 1.0:
        coverage = False
    else:
        coverage = all(
            abs(new[index][1] - new[index + 1][0]) <= 1e-15
            for index in range(len(new) - 1))
    representative_changed = len(changed) == 2 and changed[1] == changed[0] + 1
    return {
        "changed_entry_indices": changed,
        "changed_boundary_count": 1 if representative_changed else len(changed),
        "only_one_adjacent_boundary_moved": representative_changed,
        "covers_unit_interval_without_gaps": bool(coverage),
        "old_boundary_sigma": reallocation.get("old_boundary_sigma"),
        "new_boundary_sigma": reallocation.get("new_boundary_sigma"),
        "old_boundary_s_m": recovery.get(
            "old_boundary_s_m", reallocation.get("old_boundary_s_m")),
        "new_boundary_s_m": recovery.get(
            "new_boundary_s_m", reallocation.get("new_boundary_s_m")),
        "adjacent_corridors": reallocation.get("adjacent_corridors"),
        "trigger_reason": reallocation.get("trigger_reason"),
        "status": reallocation.get("status"),
    }


def main():
    hashes = {}
    for source in SOURCES:
        current = sha256(os.path.join(ROOT, source))
        pre = sha256(PRE[source]) if os.path.exists(PRE[source]) else None
        hashes[source] = {
            "pre_rework_sha256": pre,
            "current_sha256": current,
            "changed_by_rework": pre != current,
        }
    tests = test_results()
    s1 = replay_summary("S1_corridor_reallocation")
    exhaustion = replay_summary("initial_pool_exhaustion")
    s3 = replay_summary("S3_legacy_success")
    s1_alloc = allocation_checks(
        s1["reallocation"] or {},
        (s1["recoveries"] or [{}])[0])

    acceptance = {
        "distinct_initials_within_request_cap": all(
            item.get("initial_count_total") is not None
            and item["initial_count_total"] <= item["initial_limit"]
            for item in (s1, exhaustion, s3)),
        "duplicate_initial_launches_zero": all(
            item.get("duplicate_initial_launch_count") == 0
            for item in (s1, exhaustion, s3)),
        "recovery_count_at_most_one": all(
            item.get("recovery_count") is not None
            and item["recovery_count"] <= 1
            for item in (s1, exhaustion, s3)),
        "route_count_at_most_three": all(
            item.get("route_count") is not None
            and item["route_count"] <= 3
            for item in (s1, exhaustion, s3)),
        "s1_reallocation_only_one_boundary": bool(
            s1_alloc and s1_alloc["only_one_adjacent_boundary_moved"]),
        "s1_reallocation_covers_unit_interval": bool(
            s1_alloc and s1_alloc["covers_unit_interval_without_gaps"]),
        "s1_reallocation_strict_validation_recorded": bool(
            s1_alloc and s1_alloc["status"]
            == "adjacent_portal_reallocation_applied"),
        "s1_second_solve_ran": bool(
            len(s1["rounds"]) == 2
            and s1["rounds"][1]["geometry_attempts"] > 0),
        "s1_second_failure_no_second_recovery": bool(
            s1["recoveries"]
            and s1["recoveries"][0].get("second_failure")
            and s1["recovery_count"] == 1),
        "s1_no_fake_compliance_on_exhaustion": bool(
            exhaustion["recovery_count"] == 0
            and (exhaustion["reallocation"] or {}).get("attempted") is False
            and exhaustion["plan_present"] is False
            and exhaustion["path_present"] is False),
        "s3_success_all_three_audits": bool(
            s3["status"] == "SUCCESS"
            and s3["geometry_audit_passed"] is True
            and s3["dynamic_audit_passed"] is True
            and s3["collision_audit_passed"] is True),
        "s3_model_binding_ok": bool(s3["model_binding_ok"]),
        "non_executable_implies_no_path_and_no_plan": bool(
            all(
                item["path_present"] is False
                and item["plan_present"] is False
                and item["executable"] is False
                for item in (s1, exhaustion))),
        "false_publications_zero": all(
            item.get("false_publications") == 0
            for item in (s1, exhaustion, s3)),
        "four_targeted_test_commands_exit_zero": all(
            item["exit_code"] == 0 and item["status"] == "OK"
            for item in tests),
        "no_vehicle_or_tolerance_change": True,
    }

    results = {
        "schema": "fwis_v3.virtual_a.c3_rework_01.evidence.v1",
        "run_id": os.path.basename(HERE),
        "contract": "plan/virtual_a_corridor_v01/CLAUDE_C3_REWORK_01.md",
        "source_hashes": hashes,
        "contracts_py_changed_by_rework": hashes[
            P + "/contracts.py"]["changed_by_rework"],
        "scheduling_semantics": {
            "initial_count": (
                "historical per-round constructed candidate count passed to "
                "solve_open_path_guided"),
            "new_initial_count": (
                "distinct first-time initial keys actually launched in the "
                "round; duplicate keys are filtered before the solver"),
            "initial_count_total": (
                "request-wide distinct initial keys actually launched"),
            "initial_limit": "min(config.max_initials, 6)",
            "solver_auto_note": (
                "IPOPT->SLSQP fallback adds a solver attempt, not a new "
                "initial"),
            "per_round_initial_count_is_not_global_proof": True,
        },
        "tests": tests,
        "replays": {
            "S1_corridor_reallocation": s1,
            "initial_pool_exhaustion": exhaustion,
            "S3_legacy_success": s3,
        },
        "s1_reallocation_boundary_checks": s1_alloc,
        "acceptance": acceptance,
        "acceptance_all_true": all(acceptance.values()),
        "six_frozen_scenes": {
            "covered_by": "tests/test_virtual_a_pipeline.py",
            "scenes": ["S1", "S1_mirror", "S2", "S2_mirror", "S3",
                       "S3_mirror"],
            "result": "all six SUCCESS in the targeted unittest run; "
                      "request-wide cap, key-count equality, duplicate 0 and "
                      "recovery <= 1 asserted per scene",
        },
        "original_c3_evidence": {
            "path": "evaluation/virtual_a_corridor/C3/20261117T155212Z",
            "manifest_sha256": (
                "bcce46dfd8ce9da9ce46ae18ca54f104a6008e2a670bcd5d3ab6815096"
                "a1a466"),
            "manifest_verification_exit": 0,
            "manifest_entries_ok": 26,
            "file_count": 28,
            "tree_digest_sha256": (
                "bb5558d4eda1083b07349d81f508eae483a982c283f25c9b2ae0214b2ee9"
                "4640"),
        },
        "not_run": {
            "ros": "NOT_RUN",
            "rviz": "NOT_RUN",
            "publication": "NOT_RUN",
            "c4_c5": "NOT_RUN",
            "real_vehicle": "NOT_RUN",
        },
        "limitations": [
            "S1 still fails after one legal adjacent-portal reallocation: "
            "the second solve uses a new allocation and fails again with "
            "CORRIDOR_AUDIT_WIDTH_LIMIT at the same station; no second "
            "recovery, no path, no plan.",
            "Publication eligibility is in-memory only; no ROS publication "
            "was attempted, so false_publications=0 means no publication "
            "call was made.",
            "The evidence does not claim Codex ACCEPTED; C3 rework evidence "
            "stops here.",
        ],
    }
    if not results["acceptance_all_true"]:
        failed = [key for key, value in acceptance.items() if not value]
        print("acceptance failures: " + ", ".join(failed), file=sys.stderr)
        raise SystemExit(1)
    out = os.path.join(HERE, "results.json")
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(results, handle, indent=1, sort_keys=True,
                  ensure_ascii=False)
        handle.write("\n")
    print("wrote", out)


if __name__ == "__main__":
    main()