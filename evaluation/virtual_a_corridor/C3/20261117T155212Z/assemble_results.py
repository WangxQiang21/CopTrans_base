"""Assemble the C3 results.json from the recorded logs and case files.

Evidence harness only.  It parses the three required unittest logs, the C2/C3
corridor comparison, and the source hashes; it does not run the planner.
"""
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path

EV = Path(__file__).resolve().parent
ROOT = EV.parents[3]
REL = "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a"
PRE = Path("/tmp/c3_pre")

TEST_FILES = (
    "test_virtual_a_corridor_recovery.py",
    "test_virtual_a_optimizer.py",
    "test_virtual_a_pipeline.py",
)
CHANGED = (
    REL + "/pipeline.py",
    REL + "/optimizer.py",
    REL + "/contracts.py",
    "tests/test_virtual_a_pipeline.py",
    "tests/test_virtual_a_corridor_recovery.py",
)
PRE_FILES = {
    REL + "/pipeline.py": PRE / REL / "pipeline.py",
    REL + "/optimizer.py": PRE / REL / "optimizer.py",
    REL + "/contracts.py": PRE / REL / "contracts.py",
    "tests/test_virtual_a_pipeline.py":
        PRE / "tests/test_virtual_a_pipeline.py",
    "tests/test_virtual_a_optimizer.py":
        PRE / "tests/test_virtual_a_optimizer.py",
    "tests/virtual_a_g4_support.py":
        PRE / "tests/virtual_a_g4_support.py",
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pre_c3_hashes():
    """Hash table recorded in start_state.log before C3 code changes."""
    text = (EV / "start_state.log").read_text(encoding="utf-8")
    section = text.split("--- hashes ---", 1)[1]
    hashes = {}
    for line in section.splitlines():
        parts = line.split()
        if len(parts) == 2 and re.fullmatch(r"[0-9a-f]{64}", parts[0]):
            hashes[parts[1]] = parts[0]
    return hashes


def parse_test_log(path):
    text = Path(path).read_text(encoding="utf-8")
    match = re.search(r"Ran (\d+) tests? in ([\d.]+)s", text)
    if match is None:
        return {"test_count": None, "wall_s": None,
                "outcome": "UNPARSED", "log": path.name}
    outcome = "OK" if re.search(r"^OK$", text, re.M) else "FAILED"
    return {
        "test_count": int(match.group(1)),
        "wall_s": float(match.group(2)),
        "outcome": outcome,
        "log": str(Path(path).relative_to(EV)),
    }


def main():
    tests = {}
    for name in TEST_FILES:
        tests[name] = parse_test_log(EV / "test_logs" / name.replace(
            ".py", ".log"))
    comparison = json.loads((
        EV / "corridor_cases" / "C2_vs_C3.json").read_text(encoding="utf-8"))
    legacy = json.loads((
        EV / "corridor_cases" / "S3_legacy_c3.json").read_text(
            encoding="utf-8"))
    baseline = pre_c3_hashes()
    hashes = {}
    for name in CHANGED:
        pre = baseline.get(name)
        hashes[name] = {
            "current_sha256": sha256(ROOT / name),
            "pre_c3_base_sha256": pre,
            "new_in_c3": pre is None,
            "pre_c3_base_verified": (
                pre is not None and name in PRE_FILES
                and sha256(PRE_FILES[name]) == pre),
        }
    results = {
        "schema": "c3_scheduling_single_recovery_v1",
        "stage": "C3",
        "run_id": EV.name,
        "generated_at_utc": datetime.now(timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%SZ"),
        "base": {
            "head": "9a19fd3e22d33188b13795f695757c3e4a36909f",
            "branch": "feature/virtual-rigid-formation-a",
            "worktree": str(ROOT),
            "uncommitted": True,
            "branch_created": False,
            "commit_created": False,
            "pushed": False,
            "pre_c3_tree_reconstruction": (
                "HEAD blobs + C2-A/C2-B/C2-B-rework patches in /tmp/c3_pre; "
                "sha256 equals evaluation/.../C3/.../start_state.log"),
        },
        "prerequisite": {
            "c2b_rework_evidence":
                "evaluation/virtual_a_corridor/C2B_REWORK_01/20261117T151325Z/",
            "user_order": "进入C3任务 (2026-11-17)",
            "codex_accepted_claimed": False,
        },
        "scheduling_caps": {
            "route_candidates_max": 3,
            "initial_values_max_per_solve": 6,
            "recoveries_max_per_request": 1,
            "request_soft_budget_s": 60.0,
            "single_solve_soft_budget_s": 30.0,
            "route_generation_soft_budget_s": 8.0,
            "hard_real_time_guarantee": False,
            "budget_overshoot_possible": True,
        },
        "tests": tests,
        "frozen_scenes": {
            "test": "test_six_frozen_scenes_pass_all_three_audits_autonomously",
            "scenes": ["S1", "S1_mirror", "S2", "S2_mirror", "S3",
                       "S3_mirror"],
            "outcome": tests["test_virtual_a_pipeline.py"]["outcome"],
            "evidence": "test_logs/test_virtual_a_pipeline.log",
        },
        "corridor_same_input_comparison": comparison,
        "corridor_supplementary_legacy_s3": {
            "guidance_mode": legacy["guidance_mode"],
            "solver": legacy["solver"],
            "status": legacy["status"],
            "executable": legacy["executable"],
            "path_present": legacy["path_present"],
            "plan_present": legacy["plan_present"],
            "path_length_m": legacy["path_length_m"],
            "wall_time_s": legacy["wall_time_s"],
            "route_count": legacy["route_count"],
            "initial_count": legacy["initial_count"],
            "recovery_count": legacy["recovery_count"],
            "first_failure_layer": legacy["first_failure_layer"],
            "last_reason": legacy["last_reason"],
            "recoveries": legacy["recoveries"],
            "attempt_count": len(legacy["attempts"]),
            "model_sha256": legacy.get("model_sha256"),
            "plan_model_sha256": legacy.get("plan_model_sha256"),
            "plan_request_sha256": legacy.get("plan_request_sha256"),
            "plan_planning_context_sha256":
                legacy.get("plan_planning_context_sha256"),
            "model_binding_ok": legacy.get("model_binding_ok"),
            "round_initial_modes": [
                record["initial_mode"] for record in legacy["rounds"]],
            "round_initial_counts": [
                record["initial_count"] for record in legacy["rounds"]],
            "geometry_audit_passed": legacy["geometry_audit_passed"],
            "dynamic_audit_passed": legacy["dynamic_audit_passed"],
            "collision_audit_passed": legacy["collision_audit_passed"],
            "checks": legacy["checks"],
            "evidence_file":
                "corridor_cases/S3_legacy_c3.json",
            "note": (
                "Supplementary positive recovery run.  The first round fails "
                "collision numerically; the single endpoint-only recovery "
                "supplies the G2 no-route initial family, and the retimed "
                "plan passes all three independent audits."),
        },
        "source_hashes": hashes,
        "false_publications": comparison["false_publications"],
        "not_run": [
            "ROS nodes, ROS master, RViz display and adapter publication",
            "real vehicle / hardware",
            "corridor mirror scenes under C3 (legacy mirrors are covered by "
            "the six frozen-scene test)",
            "scipy corridor variants under C3",
        ],
        "not_implemented": [
            "fixed-sigma failure adjacent-portal reallocation: corridor.py is "
            "outside the TASK_C3 allowed file set; the allocation-limit path "
            "fails closed as FIXED_ALLOCATION_LIMIT without relaxing audit",
            "C-degenerate single seed split: not implemented; the contract's "
            "alternative action (select another already generated route) is "
            "implemented for corridor build/guide/allocation/chain-audit "
            "failure",
            "corridor hard containment failure has no recovery action in the "
            "TASK_C3 list and fails closed without spending the recovery slot",
        ],
        "test_coverage_map": {
            "single_box_left_right_and_open_dogleg_dedup":
                "test_open_grid_merges_slight_doglegs_into_one_candidate; "
                "test_single_box_keeps_both_obstacle_separated_sides",
            "endpoint_not_attached_to_route0":
                "test_corridor_endpoint_initials_are_not_pulled_to_route_zero; "
                "test_initial_slots_are_one_per_route_plus_endpoint_shapes",
            "fake_solver_success_audit_failure":
                "test_fake_solver_success_but_audit_rejection_spends_no_recovery",
            "simulated_timeout_no_fallback":
                "test_simulated_timeout_never_starts_the_slsqp_fallback",
            "obb_failure_switches_route_once":
                "test_obb_failure_switches_to_one_untried_route_then_stops",
            "recovery_fails_no_plan":
                "test_numerical_failure_spends_one_endpoint_only_recovery; "
                "test_obb_failure_switches_to_one_untried_route_then_stops",
            "audited_candidate_preserved_through_later_timeout":
                "test_timeout_preserves_only_a_previously_audited_candidate "
                "(existing test_virtual_a_optimizer.py SoftDeadlineTests)",
            "fake_solver_budget_boundary":
                "test_budget_exhaustion_between_route_generation_and_solve",
            "one_real_success_and_one_real_failure":
                "test_real_success_and_real_failure_requests",
        },
    }
    (EV / "results.json").write_text(
        json.dumps(results, indent=1, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "tests": tests,
        "false_publications": results["false_publications"],
    }, indent=1, sort_keys=True))


if __name__ == "__main__":
    main()