"""Assemble the C2-B results.json from the per-case pipeline JSON outputs.

Reads every <scene>_<mode>.json written by run_pipeline_case.py plus the
recorded test logs and git state, and writes results.json.  Evidence only.
"""
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def _load(name):
    path = HERE / name
    if not path.exists():
        return None
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def _case_summary(document):
    if document is None:
        return None
    return {
        "scene": document["scene"],
        "guidance_mode": document["guidance_mode"],
        "solver": document["planner_config"].get("solver"),
        "status": document["status"],
        "reason": document["reason"],
        "detail": document["detail"],
        "feasible": document["feasible"],
        "executable": document["executable"],
        "path_present": document["path_present"],
        "plan_present": document["plan_present"],
        "path_length_m": document["path_length_m"],
        "plan_kind": document["plan_kind"],
        "request_sha256": document["request_sha256"],
        "planning_context_sha256": document["planning_context_sha256"],
        "grid_sha256": document["grid_sha256"],
        "wall_time_s": document["wall_time_s"],
        "measured_wall_s": document["measured_wall_s"],
        "geometry_audit_passed": document["geometry_audit_passed"],
        "dynamic_audit_passed": document["dynamic_audit_passed"],
        "collision_audit_passed": document["collision_audit_passed"],
        "checks": document["checks"],
        "corridor_rounds": [
            {
                "round_index": round_["round_index"],
                "route_source": round_["route_source"],
                "chain_digest": round_["chain_digest"],
                "region_count": round_["region_count"],
                "region_rows": round_["region_rows"],
                "allocation": round_["allocation"],
                "effective_margin_m": round_["effective_margin_m"],
                "chain_audit_passed": round_["chain_audit_passed"],
                "guide_source": round_["guide_source"],
                "guide_points": round_["guide_points"],
                "containment_passed": (
                    round_["containment"] or {}).get("passed"),
                "containment_reason": (
                    round_["containment"] or {}).get("reason"),
                "containment_samples": (
                    round_["containment"] or {}).get("sample_count"),
                "containment_max_point_residual_m": (
                    round_["containment"] or {}).get(
                        "max_point_residual_m"),
                "containment_max_bound_residual_m": (
                    round_["containment"] or {}).get(
                        "max_bound_residual_m"),
            }
            for round_ in document["corridor_rounds"]],
        "search_rounds": document["search_rounds"],
        "attempts": [
            {
                "attempt_index": attempt["attempt_index"],
                "initial_source": attempt["initial_source"],
                "status": attempt["status"],
                "solver": attempt["solver"],
                "solver_status": attempt["solver_status"],
                "corridor_residual_m": attempt["corridor_residual_m"],
                "rejection_reason": attempt["rejection_reason"],
            }
            for attempt in document["attempts"]],
        "metrics": {
            key: value for key, value in document["metrics"].items()
            if key != "collision_margin_m"},
        "kappa_peaks_per_vehicle": document.get("kappa_peaks_per_vehicle"),
        "kappa_reference_peak": document.get("kappa_reference_peak"),
    }


def _tail(path, lines=3):
    target = HERE / path
    if not target.exists():
        return None
    text = target.read_text(encoding="utf-8").strip().splitlines()
    return text[-lines:]


def main():
    cases = []
    for name in sorted(HERE.glob("*_corridor.json")):
        cases.append(_case_summary(_load(name.name)))
    for name in sorted(HERE.glob("*_corridor_scipy.json")):
        cases.append(_case_summary(_load(name.name)))
    for name in sorted(HERE.glob("*_legacy_route.json")):
        cases.append(_case_summary(_load(name.name)))
    cases = [case for case in cases if case is not None]
    corridor_cases = [case for case in cases
                      if case["guidance_mode"] == "corridor"]

    def _fully_valid(case):
        containment_ok = any(
            round_["containment_passed"] and
            round_["containment_reason"] == "CORRIDOR_CONTAINMENT_VERIFIED"
            for round_ in case["corridor_rounds"])
        return (case["status"] == "SUCCESS" and case["feasible"]
                and case["executable"] and case["path_present"]
                and case["plan_present"]
                and case["geometry_audit_passed"] is True
                and case["dynamic_audit_passed"] is True
                and case["collision_audit_passed"] is True
                and containment_ok)

    # A false publication is any corridor case that is not fully valid yet
    # still reports SUCCESS or carries a path/plan payload.
    false_publications = [
        case for case in corridor_cases
        if not _fully_valid(case)
        and (case["status"] == "SUCCESS" or case["path_present"]
             or case["plan_present"])]
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=str(ROOT),
        capture_output=True, text=True).stdout.strip()
    branch = subprocess.run(
        ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=str(ROOT),
        capture_output=True, text=True).stdout.strip()
    document = {
        "schema": "virtual_rigid_a.c2b.pipeline.v1",
        "stage": "C2-B",
        "generated_at_utc": subprocess.run(
            ["date", "-u", "+%Y-%m-%dT%H:%M:%SZ"],
            capture_output=True, text=True).stdout.strip(),
        "run_id": "20261117T135647Z",
        "git": {
            "head": head,
            "branch": branch,
            "commit_created": False,
            "branch_created": False,
            "pushed": False,
        },
        "environment": {
            "file": "environment.txt",
            "python": "3.8.10",
            "numpy": "1.24.4",
            "scipy": "1.10.1",
            "casadi": "3.7.2",
            "ipopt_plugin_available": True,
        },
        "tests": {
            "targeted_suites": {
                "command": (
                    "PYTHONPATH=$PWD/catkin_ws/src/fwis_v3_optimizer/src:"
                    "$PWD/tests:$PWD python3 -B -m unittest "
                    "tests.test_virtual_a_corridor_optimizer "
                    "tests.test_virtual_a_corridor tests.test_virtual_a_optimizer "
                    "tests.test_virtual_a_trajectory tests.test_virtual_a_pipeline"),
                "exit_code": _tail("test_logs/targeted_suites.exit", 1),
                "log_tail": _tail("test_logs/targeted_suites.log", 4),
            },
            "c2b_module_verbose": {
                "command": (
                    "PYTHONPATH=$PWD/catkin_ws/src/fwis_v3_optimizer/src:"
                    "$PWD/tests:$PWD python3 -B -m unittest "
                    "tests.test_virtual_a_corridor_optimizer -v"),
                "exit_code": _tail(
                    "test_logs/test_virtual_a_corridor_optimizer_verbose.exit", 1),
                "log_tail": _tail(
                    "test_logs/test_virtual_a_corridor_optimizer_verbose.log", 4),
            },
        },
        "pipeline_cases": cases,
        "corridor_case_count": len(corridor_cases),
        "corridor_successes": [
            case["scene"] for case in corridor_cases
            if case["status"] == "SUCCESS"],
        "corridor_rejections": [
            {"scene": case["scene"], "status": case["status"],
             "reason": case["reason"],
             "path_present": case["path_present"],
             "plan_present": case["plan_present"]}
            for case in corridor_cases if case["status"] != "SUCCESS"],
        "false_publications": false_publications,
        "pipeline_exit_codes": _tail("pipeline_exit_codes.log", 40),
    }
    with open(HERE / "results.json", "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=1, sort_keys=True, default=str)
        handle.write("\n")
    print(json.dumps({
        "cases": len(cases),
        "corridor_cases": len(corridor_cases),
        "corridor_successes": document["corridor_successes"],
        "false_publications": len(false_publications),
    }, sort_keys=True))


if __name__ == "__main__":
    main()