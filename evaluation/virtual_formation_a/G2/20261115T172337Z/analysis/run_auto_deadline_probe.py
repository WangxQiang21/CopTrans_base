"""Record deterministic evidence for the G2 auto-solver deadline paths.

The probe exercises the production ``solve_open_path`` control flow.  Two
cases use genuine IPOPT/SLSQP results; the remaining cases mock only the
solver boundary result or raise the primary exception so that control-flow
branches can be hit deterministically.  A timed-out iterate is expected never
to reach ``audit_geometry``.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path
import sys
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
RUN = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(
    ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape  # noqa: E402
from fwis_v3_optimizer.formation_retiming import (  # noqa: E402
    FormationTimingLimits,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    GoalPolicy,
    InitialAssumption,
    PlannerConfig,
    VirtualARequest,
)
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    optimizer as optimizer_module,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (  # noqa: E402
    make_initial_guesses,
    solve_open_path,
)

FIXTURE = json.loads(
    (ROOT / "tests/fixtures/virtual_a/open_requests.json").read_text(
        encoding="utf-8"))
BASE = FIXTURE["base"]


def planner_config(**changes):
    values = dict(FIXTURE["planner_config"])
    values.update(changes)
    return PlannerConfig(**values)


def request_e1():
    record = FIXTURE["requests"]["E1"]
    model = FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in BASE["model"]["offsets"]])
    shapes = {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in BASE["model"]["vehicle_shapes"]}
    return VirtualARequest(
        plan_id=record["plan_id"],
        formation_id=record["formation_id"],
        model=model,
        vehicle_shapes=shapes,
        start_qf=FormationPose(**BASE["start_qf"]),
        start_headings={
            int(key): float(value)
            for key, value in BASE["start_headings"].items()},
        goal_qf=FormationPose(
            record["goal_qf"]["x"], record["goal_qf"]["y"],
            record["goal_qf"]["psi"]),
        initial=InitialAssumption(**BASE["initial"]),
        goal_policy=GoalPolicy(record["goal_policy"]),
        limits=FormationTimingLimits(**BASE["limits"]),
    )


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def timeout_result(guess, stage, status, success=False):
    return {
        "solver": optimizer_module.SOLVER_IPOPT,
        "z": np.asarray(guess.vector, dtype=float).copy(),
        "objective": 0.0,
        "solver_status": status,
        "success": success,
        "iterations": 3,
        "detail": "forced {} timeout".format(stage),
        "wall_time_s": 0.01,
        "soft_deadline_triggered": True,
        "deadline_trigger_stage": stage,
        "remaining_budget_s": 1e-9,
    }


def failed_ipopt(status="Solve_Failed", detail="forced ordinary failure"):
    return {
        "solver": optimizer_module.SOLVER_IPOPT,
        "z": None,
        "objective": None,
        "solver_status": status,
        "success": False,
        "iterations": 2,
        "detail": detail,
        "wall_time_s": 0.01,
        "soft_deadline_triggered": False,
        "deadline_trigger_stage": "",
        "remaining_budget_s": 29.99,
    }


def attempts_summary(result):
    rows = []
    for attempt in result.attempts:
        rows.append({
            "attempt_index": attempt.attempt_index,
            "solver": attempt.solver,
            "solver_status": attempt.solver_status,
            "status": attempt.status,
            "iterations": attempt.iterations,
            "soft_deadline_triggered": attempt.soft_deadline_triggered,
            "deadline_trigger_stage": attempt.deadline_trigger_stage,
            "rejection_reason": attempt.rejection_reason,
            "detail": attempt.detail,
            "audit_present": attempt.audit is not None,
            "audit_passed": (
                None if attempt.audit is None else attempt.audit.passed),
        })
    return rows


def result_summary(result):
    return {
        "status": result.status,
        "reason": result.reason,
        "detail": result.detail,
        "solver_used": result.solver_used,
        "path_present": result.path is not None,
        "audit_present": result.audit is not None,
        "audit_passed": (
            None if result.audit is None else result.audit.passed),
        "deadline_exhausted": result.deadline_exhausted,
        "deadline_summary": result.deadline_summary,
        "attempts": attempts_summary(result),
    }


def case_record(checks, result, **extra):
    record = dict(extra)
    record.update({
        "checks": checks,
        "checks_pass": all(check["pass"] for check in checks),
        "result": result_summary(result),
    })
    return record


def main():
    request = request_e1()
    cases = {}

    # A. auto + IPOPT timeout: any SLSQP invocation is a hard failure.
    config = planner_config(solver="auto", max_initials=1)
    guess = make_initial_guesses(request, config)[0]
    fallback_calls = []
    audit_calls = []
    original_audit = optimizer_module.audit_geometry

    def forbidden_slsqp(*args, **kwargs):
        fallback_calls.append(1)
        raise AssertionError("SLSQP was invoked after an IPOPT timeout")

    def recording_audit(*args, **kwargs):
        audit_calls.append(1)
        return original_audit(*args, **kwargs)

    with mock.patch.object(
            optimizer_module, "_solve_with_ipopt",
            side_effect=lambda *a, **k: timeout_result(
                guess, "ipopt_max_wall_time", "Maximum_WallTime_Exceeded")):
        with mock.patch.object(
                optimizer_module, "_solve_with_scipy",
                side_effect=forbidden_slsqp):
            with mock.patch.object(
                    optimizer_module, "audit_geometry",
                    side_effect=recording_audit):
                result = solve_open_path(request, config)
    checks = [
        {"name": "status", "pass": result.status == "BUDGET_EXHAUSTED"},
        {"name": "deadline_exhausted", "pass": result.deadline_exhausted},
        {"name": "no_fallback", "pass": not fallback_calls},
        {"name": "no_audit", "pass": not audit_calls},
        {"name": "no_path", "pass": result.path is None},
        {"name": "timeout_attempt", "pass": (
            len(result.attempts) == 1
            and result.attempts[0].status == "SOLVER_TIMEOUT")},
        {"name": "trigger_stage", "pass": (
            result.deadline_summary.get("trigger_stage")
            == "ipopt_max_wall_time")},
        {"name": "timeout_status", "pass": (
            result.deadline_summary.get("timeout_solver_status")
            == "Maximum_WallTime_Exceeded")},
    ]
    cases["auto_ipopt_timeout"] = case_record(
        checks, result, fallback_calls=len(fallback_calls),
        audit_calls=len(audit_calls))

    # B. auto + one genuine audited candidate, then IPOPT timeout.
    config = planner_config(solver="auto", max_initials=2)
    guess = make_initial_guesses(request, config)[0]
    packet = optimizer_module._search_packet(request, config)
    real = optimizer_module._solve_with_ipopt(request, config, packet, guess)
    fallback_calls = []
    audit_calls = []
    calls = {"count": 0}

    def first_success_then_timeout(_request, _config, _packet, _guess,
                                   wall_limit_s=None):
        calls["count"] += 1
        if calls["count"] == 1:
            return real
        return timeout_result(
            _guess, "ipopt_max_wall_time", "Maximum_WallTime_Exceeded")

    with mock.patch.object(
            optimizer_module, "_solve_with_ipopt",
            side_effect=first_success_then_timeout):
        with mock.patch.object(
                optimizer_module, "_solve_with_scipy",
                side_effect=forbidden_slsqp):
            with mock.patch.object(
                    optimizer_module, "audit_geometry",
                    side_effect=recording_audit):
                result = solve_open_path(request, config)
    checks = [
        {"name": "real_ipopt_success", "pass": bool(real["success"])},
        {"name": "status", "pass": result.status == "GEOMETRY_FEASIBLE"},
        {"name": "audit_passed", "pass": (
            result.audit is not None and result.audit.passed)},
        {"name": "deadline_exhausted", "pass": result.deadline_exhausted},
        {"name": "no_fallback", "pass": not fallback_calls},
        {"name": "audit_calls", "pass": audit_calls == [1]},
        {"name": "candidate_preserved", "pass": (
            result.deadline_summary.get("audited_candidate_preserved")
            is True)},
        {"name": "attempt_sequence", "pass": (
            [attempt.status for attempt in result.attempts]
            == ["SOLVER_SUCCESS_AUDITED", "SOLVER_TIMEOUT"])},
        {"name": "timeout_status", "pass": (
            result.deadline_summary.get("timeout_solver_status")
            == "Maximum_WallTime_Exceeded")},
    ]
    cases["auto_timeout_after_audited_candidate"] = case_record(
        checks, result, fallback_calls=len(fallback_calls),
        audit_calls=len(audit_calls), real_ipopt_iterations=real["iterations"])

    # C. auto + primary exception, controllable clock reaches budget zero
    # immediately before the fallback boundary.
    config = planner_config(
        solver="auto", max_initials=1, max_seconds=10.0)
    fallback_calls = []
    audit_calls = []
    fake_time = mock.Mock()
    fake_time.monotonic.side_effect = [
        0.0, 0.0, 0.1, 0.1, 0.2, 10.1, 10.2, 10.3, 10.4, 10.5]

    def failing_ipopt(*args, **kwargs):
        raise RuntimeError("forced non-timeout IPOPT exception")

    with mock.patch.object(optimizer_module, "time", fake_time):
        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=failing_ipopt):
            with mock.patch.object(
                    optimizer_module, "_solve_with_scipy",
                    side_effect=forbidden_slsqp):
                with mock.patch.object(
                        optimizer_module, "audit_geometry",
                        side_effect=recording_audit):
                    result = solve_open_path(request, config)
    checks = [
        {"name": "status", "pass": result.status == "BUDGET_EXHAUSTED"},
        {"name": "deadline_exhausted", "pass": result.deadline_exhausted},
        {"name": "no_fallback", "pass": not fallback_calls},
        {"name": "no_audit", "pass": not audit_calls},
        {"name": "no_path", "pass": result.path is None},
        {"name": "exception_attempt", "pass": (
            len(result.attempts) == 1
            and result.attempts[0].status == "SOLVER_FAILED"
            and result.attempts[0].solver_status == "IPOPT_EXCEPTION")},
        {"name": "exception_detail", "pass": (
            "forced non-timeout IPOPT exception"
            in result.attempts[0].detail)},
        {"name": "trigger_stage", "pass": (
            result.deadline_summary.get("trigger_stage")
            == "pre_fallback_boundary")},
        {"name": "trigger_status", "pass": (
            result.deadline_summary.get("trigger_solver_status")
            == "IPOPT_EXCEPTION")},
        {"name": "not_a_solver_timeout", "pass": (
            "timeout_solver_status" not in result.deadline_summary)},
    ]
    cases["auto_exception_budget_exhausted"] = case_record(
        checks, result, fallback_calls=len(fallback_calls),
        audit_calls=len(audit_calls))

    # D. auto + ordinary IPOPT failure, genuine SLSQP fallback and audit.
    config = planner_config(solver="auto", max_initials=1)
    audit_calls = []
    with mock.patch.object(
            optimizer_module, "_solve_with_ipopt",
            side_effect=lambda *a, **k: failed_ipopt()):
        with mock.patch.object(
                optimizer_module, "audit_geometry",
                side_effect=recording_audit):
            result = solve_open_path(request, config)
    checks = [
        {"name": "status", "pass": result.status == "GEOMETRY_FEASIBLE"},
        {"name": "audit_passed", "pass": (
            result.audit is not None and result.audit.passed)},
        {"name": "audit_calls", "pass": len(audit_calls) == 1},
        {"name": "solver_used", "pass": result.solver_used == "scipy_slsqp"},
        {"name": "attempt_sequence", "pass": (
            [attempt.status for attempt in result.attempts]
            == ["SOLVER_FAILED", "SOLVER_SUCCESS_AUDITED"])},
        {"name": "primary_failure_kept", "pass": (
            result.attempts[0].solver_status == "Solve_Failed"
            and "forced ordinary failure" in result.attempts[0].detail)},
        {"name": "fallback_label", "pass": (
            "IPOPT Solve_Failed; SLSQP fallback used"
            in result.attempts[1].detail)},
    ]
    cases["auto_ordinary_failure_real_fallback"] = case_record(
        checks, result, audit_calls=len(audit_calls))

    # E. auto + genuine SLSQP timeout feed through the fallback branch.
    config = planner_config(solver="auto", max_initials=1)
    guess = make_initial_guesses(request, config)[0]
    packet = optimizer_module._search_packet(request, config)
    real_timeout = optimizer_module._solve_with_scipy(
        request, config, packet, guess, wall_limit_s=1e-9)
    audit_calls = []
    with mock.patch.object(
            optimizer_module, "_solve_with_ipopt",
            side_effect=lambda *a, **k: failed_ipopt(
                detail="forced IPOPT failure before SLSQP timeout")):
        with mock.patch.object(
                optimizer_module, "_solve_with_scipy",
                side_effect=lambda *a, **k: real_timeout):
            with mock.patch.object(
                    optimizer_module, "audit_geometry",
                    side_effect=recording_audit):
                result = solve_open_path(request, config)
    checks = [
        {"name": "real_slsqp_timeout", "pass": (
            real_timeout["solver_status"] == "SLSQP_SOFT_DEADLINE"
            and real_timeout["soft_deadline_triggered"] is True)},
        {"name": "status", "pass": result.status == "BUDGET_EXHAUSTED"},
        {"name": "deadline_exhausted", "pass": result.deadline_exhausted},
        {"name": "no_audit", "pass": not audit_calls},
        {"name": "no_path", "pass": result.path is None},
        {"name": "attempt_sequence", "pass": (
            [attempt.status for attempt in result.attempts]
            == ["SOLVER_FAILED", "SOLVER_TIMEOUT"])},
        {"name": "timeout_status", "pass": (
            result.deadline_summary.get("timeout_solver_status")
            == "SLSQP_SOFT_DEADLINE")},
        {"name": "trigger_stage", "pass": (
            result.deadline_summary.get("trigger_stage")
            == "slsqp_iteration_callback")},
        {"name": "solver_used", "pass": result.solver_used == "scipy_slsqp"},
    ]
    cases["auto_slsqp_timeout"] = case_record(
        checks, result, audit_calls=len(audit_calls),
        real_slsqp_iterations=real_timeout["iterations"])

    document = {
        "schema": "virtual_rigid_a.g2.auto_deadline_probe.v1",
        "stage": "G2",
        "run_id": RUN.name,
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "scope": "geometry-only auto-solver deadline control flow",
        "source_hashes": {
            "optimizer.py": sha256(
                ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
                "formation_continuous_a/optimizer.py"),
            "contracts.py": sha256(
                ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
                "formation_continuous_a/contracts.py"),
            "tests/test_virtual_a_optimizer.py": sha256(
                ROOT / "tests/test_virtual_a_optimizer.py"),
        },
        "cases": cases,
        "all_checks_pass": all(
            record["checks_pass"] for record in cases.values()),
        "notes": [
            "The timeout cases mock solver boundary results; the test suite "
            "also exercises genuine IPOPT and SLSQP timeout wrappers.",
            "The ordinary-failure fallback case uses a genuine SLSQP solve and "
            "the production independent audit.",
            "No timed-out iterate appears in an audit.",
        ],
    }
    target = RUN / "analysis" / "auto_deadline_probe.json"
    target.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "PASS" if document["all_checks_pass"] else "FAIL",
        "run_id": RUN.name,
        "cases": {
            name: {
                "checks_pass": record["checks_pass"],
                "status": record["result"]["status"],
                "attempts": [
                    [attempt["solver"], attempt["solver_status"],
                     attempt["status"]]
                    for attempt in record["result"]["attempts"]],
                "trigger_stage": record["result"]["deadline_summary"].get(
                    "trigger_stage"),
            }
            for name, record in cases.items()},
        "output": str(target.relative_to(ROOT)),
        "output_sha256": sha256(target),
    }, indent=2, sort_keys=True))
    return 0 if document["all_checks_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())