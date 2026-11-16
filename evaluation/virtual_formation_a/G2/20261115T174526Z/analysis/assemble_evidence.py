"""Assemble the G2 acceptance summaries from the frozen benchmark artifacts.

The assembler does not solve anything and does not import the optimiser.  It
reads ``benchmark/results.json`` (the production solve records), the benchmark
summary and the frozen fixture manifest, checks the required acceptance
invariants, and writes:

* ``results.json``: original requests, endpoints, profiles, audits and walls;
* ``analysis/attempts.json``: every automatic initial guess and every attempt;
* ``analysis/acceptance_summary.json``: compact quantitative acceptance table,
  test-log evidence and negative-fixture coverage.

A failure of any invariant aborts without writing a PASS-looking summary.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path
import re

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
BENCHMARK_DIR = RUN / "benchmark"
RESULT_PATH = BENCHMARK_DIR / "results.json"
SUMMARY_PATH = BENCHMARK_DIR / "summary.json"
FROZEN_DIR = RUN / "frozen"
FROZEN_MANIFEST = FROZEN_DIR / "frozen_manifest.json"
NAMES = ("E1", "E2", "E3", "E4", "E5", "E6",
         "STRAIGHT_LINE", "STRAIGHT_POLICY")
POSITION_TOL = 0.005
HEADING_TOL = math.radians(0.2)
RIGIDITY_TOL = 1e-6
POSITION_BUDGET = 1e-7
ANGLE_BUDGET = 1e-9
AUDIT_POSITION_THRESHOLD = 5e-8
AUDIT_METHOD = "independent_composite_simpson_v1"
EXPECTED_CHECK_COUNT = 124
EXPECTED_TEST_COUNTS = {
    "audit": 31, "optimizer": 26, "g1_geometry": 20, "g1_profile": 22}
NEGATIVE_COVERAGE = [
    {
        "requirement": "non-C2 profile",
        "test": "test_non_c2_profile_rejected",
        "log": "audit",
        "expected": "NON_C2_PROFILE at C2_SECOND",
    },
    {
        "requirement": "car 1 initial heading 0.1 rad",
        "test": "test_car1_wrong_initial_heading_rejected",
        "log": "audit",
        "expected": "INITIAL_HEADING_MISMATCH vehicle_id=1",
    },
    {
        "requirement": "forged request digest",
        "test": "test_forged_request_digest_rejected",
        "log": "audit",
        "expected": "FORGED_REQUEST_DIGEST",
    },
    {
        "requirement": "forged goal checked against original request",
        "test": "test_forged_goal_is_checked_against_original_request",
        "log": "audit",
        "expected": "ENDPOINT_POSITION/ENDPOINT_HEADING",
    },
    {
        "requirement": "wrong vehicle shape",
        "test": "test_wrong_vehicle_shape_rejected",
        "log": "audit",
        "expected": "VEHICLE_SHAPE_MISMATCH",
    },
    {
        "requirement": "invalid limit",
        "test": "test_invalid_limit_rejected",
        "log": "audit",
        "expected": "INVALID_LIMITS",
    },
    {
        "requirement": "NaN goal",
        "test": "test_nan_goal_rejected_before_any_geometry_claim",
        "log": "audit",
        "expected": "INVALID_REQUEST",
    },
    {
        "requirement": "static-wheel whole-segment limit",
        "test": "test_static_wheel_limit_uses_whole_segment_envelope",
        "log": "audit",
        "expected": "STATIC_WHEEL_LIMIT",
    },
    {
        "requirement": "segment-interior curvature violation with legal nodes",
        "test": "test_segment_interior_curvature_violation_with_legal_nodes",
        "log": "audit",
        "expected": "CURVATURE_LIMIT inside the segment",
    },
    {
        "requirement": "unconfirmed enclosure not published as feasible",
        "test": "test_unconfirmed_enclosure_is_not_published_as_feasible",
        "log": "audit",
        "expected": "CURVATURE_LIMIT then AUDIT_BUDGET_EXHAUSTED",
    },
    {
        "requirement": "FREE terminal nonzero curvature, STRAIGHT rejects",
        "test": "test_terminal_nonzero_curvature_free_passes_straight_rejected",
        "log": "audit",
        "expected": "GOAL_POLICY_VIOLATION STRAIGHT_K",
    },
    {
        "requirement": "FREE terminal nonzero eta, STRAIGHT rejects",
        "test": "test_terminal_nonzero_eta_free_passes_straight_rejected",
        "log": "audit",
        "expected": "GOAL_POLICY_VIOLATION STRAIGHT_ETA",
    },
    {
        "requirement": "clamped cubic spline STRAIGHT reference",
        "test": "test_clamped_cubic_spline_straight_reference_passes",
        "log": "audit",
        "expected": "pass",
    },
    {
        "requirement": "G1 double-lane-change endpoint as straight path",
        "test": "test_g1_double_lane_change_endpoint_can_be_a_straight_path",
        "log": "audit",
        "expected": "pass",
    },
    {
        "requirement": "exact hold only for identical start and goal",
        "test": "test_hold_only_for_exactly_identical_start_and_goal",
        "log": "audit",
        "expected": "pass for exact; ENDPOINT_MISMATCH for 1e-9",
    },
    {
        "requirement": "solver exception",
        "test": "test_solver_exception_maps_to_numerical_failure",
        "log": "optimizer",
        "expected": "NUMERICAL_FAILURE/IPOPT_EXCEPTION",
    },
    {
        "requirement": "very small total budget",
        "test": "test_small_budget_returns_budget_exhausted_without_path",
        "log": "optimizer",
        "expected": "BUDGET_EXHAUSTED",
    },
    {
        "requirement": "fake solver success with wrong endpoint",
        "test": "test_fake_solver_success_wrong_endpoint_is_not_feasible",
        "log": "optimizer",
        "expected": "no GEOMETRY_FEASIBLE path",
    },
    {
        "requirement": "fake solver success with curvature violation",
        "test": "test_fake_solver_success_curvature_violation_is_not_feasible",
        "log": "optimizer",
        "expected": "no GEOMETRY_FEASIBLE path",
    },
    {
        "requirement": "production solve does not read reference/fixtures",
        "test": "test_solve_reads_no_reference_or_fixture_file",
        "log": "optimizer",
        "expected": "zero file opens",
    },
    {
        "requirement": "six guesses, fixed order, C2, request-only",
        "test": "test_six_guesses_fixed_order_reconstructible_and_c2",
        "log": "optimizer",
        "expected": "6 C2 guesses, no fixture/plan lookup",
    },
    {
        "requirement": "exact hold is NO_MOTION, near/turned is not",
        "test": "test_exact_hold_is_no_motion_but_near_or_turned_goal_is_not",
        "log": "optimizer",
        "expected": "NO_MOTION only for exact hold",
    },
    {
        "requirement": "rigid transform equivalence",
        "test": "test_rotation_and_translation_equivalence",
        "log": "optimizer",
        "expected": "both solves feasible, same length/profile",
    },
    {
        "requirement": "explicit SciPy fallback is labelled",
        "test": "test_scipy_fallback_is_recorded_explicitly",
        "log": "optimizer",
        "expected": "solver=scipy_slsqp",
    },
    {
        "requirement": "every integer audit-config field rejects bool/float/zero/negative",
        "test": "test_every_integer_field_rejects_bool_float_zero_and_negative",
        "log": "audit",
        "expected": "INVALID_AUDIT_CONFIG for each field",
    },
    {
        "requirement": "every float audit-config field rejects NaN/inf/negative/types/bool",
        "test": "test_every_float_field_rejects_nan_infinity_negative_and_types",
        "log": "audit",
        "expected": "INVALID_AUDIT_CONFIG for each field",
    },
    {
        "requirement": "audit position budget cannot exceed G1 contract",
        "test": "test_audit_budget_cannot_exceed_g1_contract",
        "log": "audit",
        "expected": "INVALID_AUDIT_CONFIG audit_position_budget_m",
    },
    {
        "requirement": "zero nonnegative tolerances are valid",
        "test": "test_zero_nonnegative_tolerances_are_a_valid_domain",
        "log": "audit",
        "expected": "valid zero domains",
    },
    {
        "requirement": "invalid audit config rejected before hold branch",
        "test": "test_invalid_config_is_rejected_before_the_hold_branch",
        "log": "audit",
        "expected": "INVALID_AUDIT_CONFIG, no hold",
    },
    {
        "requirement": "nonfinite audit intermediate fails closed",
        "test": "test_nonfinite_intermediate_audit_value_fails_closed",
        "log": "audit",
        "expected": "NONFINITE_AUDIT_VALUE",
    },
    {
        "requirement": "6 m path to 10 m goal with NaN tolerance is rejected",
        "test": "test_six_metre_path_to_ten_metre_goal_nan_tolerance_is_rejected",
        "log": "audit",
        "expected": "INVALID_AUDIT_CONFIG then ENDPOINT_POSITION_ERROR 4 m",
    },
    {
        "requirement": "nonzero fourth derivative multi-segment audit budget vs quad",
        "test": "test_nonzero_g_multisegment_against_independent_quadrature",
        "log": "audit",
        "expected": "confirmed Simpson budget within 5e-8 m",
    },
    {
        "requirement": "coarse audit grid refines until budget confirmed",
        "test": "test_coarse_grid_refines_until_the_budget_is_confirmed",
        "log": "audit",
        "expected": "refinement_steps>0 and confirmed",
    },
    {
        "requirement": "audit resource limit is BOUND_UNCONFIRMED",
        "test": "test_resource_limit_is_bound_unconfirmed_never_feasible",
        "log": "audit",
        "expected": "AUDIT_INTEGRATION_TOTAL_LIMIT",
    },
    {
        "requirement": "unreachable audit budget is BOUND_UNCONFIRMED",
        "test": "test_unreachable_budget_is_bound_unconfirmed",
        "log": "audit",
        "expected": "audit integration limit",
    },
    {
        "requirement": "IPOPT max_wall_time is effective and truthful",
        "test": "test_ipopt_max_wall_time_is_effective_and_truthful",
        "log": "optimizer",
        "expected": "Maximum_WallTime_Exceeded",
    },
    {
        "requirement": "SLSQP iteration callback stops at soft deadline",
        "test": "test_slsqp_iteration_callback_stops_and_marks_timeout",
        "log": "optimizer",
        "expected": "SLSQP_SOFT_DEADLINE",
    },
    {
        "requirement": "timed-out success flag never reaches the audit",
        "test": "test_timed_out_success_flag_never_reaches_the_audit",
        "log": "optimizer",
        "expected": "SOLVER_TIMEOUT, zero audit calls",
    },
    {
        "requirement": "timeout preserves only a previously audited candidate",
        "test": "test_timeout_preserves_only_a_previously_audited_candidate",
        "log": "optimizer",
        "expected": "GEOMETRY_FEASIBLE with audited candidate preserved",
    },
    {
        "requirement": "NaN planner tolerance cannot enter audit conversion",
        "test": "test_nan_planner_tolerance_cannot_enter_the_audit_conversion",
        "log": "optimizer",
        "expected": "INVALID_REQUEST/INVALID_CONFIG then INVALID_AUDIT_CONFIG",
    },
    {
        "requirement": "auto IPOPT timeout never starts SLSQP or audits",
        "test": "test_auto_timeout_never_starts_slsqp_or_audits",
        "log": "optimizer",
        "expected": "BUDGET_EXHAUSTED, SOLVER_TIMEOUT, zero SLSQP/audit calls",
    },
    {
        "requirement": "auto timeout after audited candidate has no fallback",
        "test": "test_auto_timeout_after_audited_candidate_never_falls_back",
        "log": "optimizer",
        "expected": "audited candidate preserved; no SLSQP; deadline true",
    },
    {
        "requirement": "auto exception with exhausted budget never starts SLSQP",
        "test": "test_auto_exception_with_exhausted_budget_never_starts_slsqp",
        "log": "optimizer",
        "expected": "IPOPT_EXCEPTION + pre_fallback_boundary exhaustion",
    },
    {
        "requirement": "auto ordinary failure with budget uses real SLSQP and audit",
        "test": "test_auto_ordinary_failure_with_budget_uses_real_slsqp_and_audit",
        "log": "optimizer",
        "expected": "SOLVER_FAILED then real SLSQP audited success; failure kept",
    },
    {
        "requirement": "auto SLSQP timeout iterate is never audited",
        "test": "test_auto_slsqp_timeout_is_recorded_without_audit",
        "log": "optimizer",
        "expected": "SOLVER_FAILED then SOLVER_TIMEOUT, zero audit calls",
    },
    {
        "requirement": "real IPOPT success residual finite and recomputable",
        "test": "test_real_ipopt_success_residual_is_finite_and_recomputable",
        "log": "optimizer",
        "expected": "finite nonnegative residual equals direct residual call",
    },
    {
        "requirement": "real SLSQP fallback success residual finite and recomputable",
        "test": "test_real_slsqp_fallback_success_residual_is_finite_and_recomputable",
        "log": "optimizer",
        "expected": "finite nonnegative fallback residual equals direct residual call",
    },
    {
        "requirement": "missing or nonfinite success vector never reports residual",
        "test": "test_missing_or_nonfinite_success_vector_never_reports_a_residual",
        "log": "optimizer",
        "expected": "CONSTRAINT_RESIDUAL_FAILURE, null residual, zero audit calls",
    },
    {
        "requirement": "residual calculation anomaly fails closed",
        "test": "test_residual_calculation_anomaly_fails_closed",
        "log": "optimizer",
        "expected": "exception/nonfinite residual is not reported, zero audit calls",
    },
]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def parse_test_log(path):
    text = Path(path).read_text(encoding="utf-8")
    ran = re.search(r"Ran (\d+) tests? in ([0-9.]+)s", text)
    if ran is None:
        raise SystemExit("cannot parse unittest summary in " + str(path))
    cases = {}
    for line in text.splitlines():
        match = re.match(
            r"^(\w+) \(([\w.]+)\) \.\.\. (ok|FAIL|ERROR|skipped.*)$", line)
        if match:
            cases[match.group(1)] = {
                "class": match.group(2),
                "outcome": match.group(3),
            }
    status = "OK" if re.search(r"^OK(?: \(|$)", text, re.M) else "FAILED"
    return {
        "path": str(Path(path).relative_to(RUN)),
        "sha256": sha256(path),
        "bytes": Path(path).stat().st_size,
        "count": int(ran.group(1)),
        "wall_time_s": float(ran.group(2)),
        "status": status,
        "cases": cases,
    }


def shortest_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def canonical_sha(value):
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")).hexdigest()


def main():
    if not (RUN / "logs" / "assemble_evidence.log").is_file():
        raise SystemExit("assemble log is missing; run through tee")
    results = load(RESULT_PATH)
    benchmark = load(SUMMARY_PATH)
    frozen = load(FROZEN_MANIFEST)
    crosscheck_path = RUN / "analysis" / "audit_budget_crosscheck.json"
    if not crosscheck_path.is_file():
        raise SystemExit("audit budget cross-check is missing")
    crosscheck = load(crosscheck_path)
    if crosscheck["all_checks_pass"] is not True:
        raise SystemExit("audit budget cross-check did not pass")
    if crosscheck["run_id"] != RUN.name:
        raise SystemExit("audit budget cross-check run id changed")
    probe_path = RUN / "analysis" / "auto_deadline_probe.json"
    if not probe_path.is_file():
        raise SystemExit("auto deadline probe is missing")
    probe = load(probe_path)
    if probe.get("all_checks_pass") is not True:
        raise SystemExit("auto deadline probe did not pass")
    if probe.get("run_id") != RUN.name:
        raise SystemExit("auto deadline probe run id changed")
    if probe.get("codex_acceptance") != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("auto deadline probe self-claims acceptance")
    if abs(crosscheck["high_precision_quadrature"]["psi"] -
           crosscheck["default_audit"]["integration"][
               "independent_endpoint"]["psi"]) > 1e-15:
        raise SystemExit("audit cross-check phase disagree at the endpoint")

    if sorted(results) != sorted(NAMES):
        raise SystemExit("benchmark result names differ from frozen names")
    if benchmark["status"] != "PASS" or benchmark["stage"] != "G2":
        raise SystemExit("benchmark is not a passing G2 run")
    if benchmark["codex_acceptance"] != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("benchmark self-claims Codex acceptance")
    checks = benchmark["checks"]
    if len(checks) != EXPECTED_CHECK_COUNT:
        raise SystemExit("benchmark check count changed")
    failed = [check["name"] for check in checks if not check["pass"]]
    if failed:
        raise SystemExit("benchmark has failed checks: {}".format(failed))

    for name, record in frozen["files"].items():
        frozen_path = FROZEN_DIR / name
        if sha256(frozen_path) != record["frozen_sha256"]:
            raise SystemExit("frozen fixture changed: " + name)
    config_record = frozen["planner_config"]
    if not config_record["reconstructed_equals_PlannerConfig"]:
        raise SystemExit("frozen planner config equality was not proven")
    config = config_record["default_values"]
    config_sha = config_record["frozen_sha256_canonical"]
    if config["max_initials"] != 6 or config["max_seconds"] != 30.0:
        raise SystemExit("default budget changed")
    if config["intervals"] != 8 or config["max_iterations"] != 120:
        raise SystemExit("default NLP settings changed")

    tests = {
        name: parse_test_log(RUN / "logs" / (name + ".log"))
        for name in EXPECTED_TEST_COUNTS}
    for name, expected in EXPECTED_TEST_COUNTS.items():
        if tests[name]["status"] != "OK":
            raise SystemExit("test log is not OK: " + name)
        if tests[name]["count"] != expected:
            raise SystemExit("test count changed for " + name)

    coverage = []
    for entry in NEGATIVE_COVERAGE:
        log = tests[entry["log"]]
        case = log["cases"].get(entry["test"])
        if case is None:
            raise SystemExit("test not found in log: " + entry["test"])
        if case["outcome"] != "ok":
            raise SystemExit("negative test did not pass: " + entry["test"])
        coverage.append(dict(
            entry, test_class=case["class"], test_outcome=case["outcome"],
            log_path=log["path"], log_sha256=log["sha256"]))

    solutions = {}
    acceptance = {}
    attempts_document = {
        "schema": "virtual_rigid_a.g2.attempts.v2",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "benchmark_results_sha256": sha256(RESULT_PATH),
        "config": config,
        "requests": {},
    }
    per_request_limits = {}
    all_ok = True
    max_wall = 0.0
    success_attempt_count = 0
    residual_nonnull_finite_count = 0
    residual_null_count = 0
    residual_values = []
    for name in NAMES:
        record = results[name]
        request = record["request"]
        result = record["result"]
        audit = result["audit"]
        goal = request["goal_qf"]
        endpoint = record["endpoint"]
        position_error = math.hypot(
            endpoint["x"] - goal["x"], endpoint["y"] - goal["y"])
        heading_error = abs(shortest_angle(
            endpoint["psi"] - goal["psi"]))
        audit_integration = audit["integration"]
        audit_budget = audit_integration["audit_integration"]
        deadline_summary = result.get("deadline_summary") or {}
        sample = audit["metrics"]["sample_audit"]
        rigidity = sample["max_pairwise_rigidity_error_m"]
        segments = audit["whole_segment_bounds"]
        max_c = max(value for segment in segments
                    for value in segment["max_abs_C_by_vehicle"].values())
        max_wheel = max(segment["max_static_wheel_angle_rad"]
                        for segment in segments)
        curvature_limit = 1.0 / request["limits"]["r_min_ackermann"]
        attempts = result["attempts"]
        wall = result["wall_time_s"]
        max_wall = max(max_wall, wall)

        if result["status"] != "GEOMETRY_FEASIBLE":
            all_ok = False
        if not result.get("path") or not audit["passed"]:
            all_ok = False
        if result["deadline_exhausted"]:
            all_ok = False
        if result["request_sha256"] != record["request_sha256"]:
            all_ok = False
        if audit["request_sha256"] != record["request_sha256"]:
            all_ok = False
        if len(attempts) != 6:
            all_ok = False
        for attempt in attempts:
            if attempt["status"] != "SOLVER_SUCCESS_AUDITED":
                all_ok = False
            if attempt["solver"] != "ipopt":
                all_ok = False
            if not attempt["candidate_audit"]["passed"]:
                all_ok = False
            if attempt["iterations"] > config["max_iterations"]:
                all_ok = False
            if attempt["status"] == "SOLVER_SUCCESS_AUDITED":
                success_attempt_count += 1
                residual = attempt["constraint_residual"]
                if residual is None:
                    residual_null_count += 1
                    all_ok = False
                elif (isinstance(residual, (int, float))
                      and not isinstance(residual, bool)
                      and math.isfinite(float(residual))
                      and float(residual) >= 0.0):
                    residual_nonnull_finite_count += 1
                    residual_values.append(float(residual))
                else:
                    all_ok = False
        if position_error > POSITION_TOL:
            all_ok = False
        if heading_error > HEADING_TOL:
            all_ok = False
        if rigidity > RIGIDITY_TOL:
            all_ok = False
        if audit_integration["position_budget_m"] > POSITION_BUDGET:
            all_ok = False
        if audit_integration["angle_budget_rad"] > ANGLE_BUDGET:
            all_ok = False
        if audit_budget["method"] != AUDIT_METHOD:
            all_ok = False
        if audit_budget["confirmed"] is not True:
            all_ok = False
        if audit_budget["fixed_threshold_m"] != AUDIT_POSITION_THRESHOLD:
            all_ok = False
        if audit_budget["total_budget_m"] > AUDIT_POSITION_THRESHOLD:
            all_ok = False
        if audit_integration["audit_position_budget_m"] > \
                AUDIT_POSITION_THRESHOLD:
            all_ok = False
        if audit_integration["audit_angle_budget_rad"] > ANGLE_BUDGET:
            all_ok = False
        if audit_integration["combined_position_budget_m"] < \
                audit_integration["position_budget_m"] + \
                audit_integration["audit_position_budget_m"]:
            all_ok = False
        if deadline_summary.get("hard_real_time_guarantee") is not False:
            all_ok = False
        if deadline_summary.get("triggered") is not False:
            all_ok = False
        if result["deadline_exhausted"] is not False:
            all_ok = False
        if max_c > curvature_limit:
            all_ok = False
        if max_wheel > request["limits"]["steer_angle_max"]:
            all_ok = False
        if abs(audit["metrics"]["max_static_wheel_angle_rad"] - max_wheel) > 1e-12:
            all_ok = False
        if wall >= config["max_seconds"]:
            all_ok = False
        if any(segment["max_depth_reached"] > 20 for segment in segments):
            all_ok = False
        if len(segments) != 8:
            all_ok = False
        if audit["scope"] != "GEOMETRY" or audit["check_count"] <= 0:
            all_ok = False
        if name == "STRAIGHT_POLICY":
            terminal = record["profile"]["knots"][-1]
            if abs(terminal["value"]) > 1e-9 or \
                    abs(terminal["derivative"]) > 1e-9:
                all_ok = False
        per_request_limits[name] = {
            "curvature_limit_1_per_m": curvature_limit,
            "static_wheel_limit_rad": request["limits"]["steer_angle_max"],
        }

        attempt_rows = []
        for attempt in attempts:
            candidate = attempt["candidate_audit"]
            attempt_rows.append({
                "attempt_index": attempt["attempt_index"],
                "initial_source": attempt["initial_source"],
                "initial_label": attempt["initial_label"],
                "initial": attempt["initial"],
                "solver": attempt["solver"],
                "solver_status": attempt["solver_status"],
                "status": attempt["status"],
                "iterations": attempt["iterations"],
                "objective": attempt["objective"],
                "constraint_residual": attempt["constraint_residual"],
                "constraint_residual_finite": (
                    attempt["constraint_residual"] is not None
                    and isinstance(attempt["constraint_residual"],
                                   (int, float))
                    and not isinstance(attempt["constraint_residual"], bool)
                    and math.isfinite(
                        float(attempt["constraint_residual"]))
                    and float(attempt["constraint_residual"]) >= 0.0),
                "endpoint_error_m": attempt["endpoint_error_m"],
                "heading_error_rad": attempt["heading_error_rad"],
                "wall_time_s": attempt["wall_time_s"],
                "rejection_reason": attempt["rejection_reason"],
                "soft_deadline_triggered": attempt.get(
                    "soft_deadline_triggered", False),
                "deadline_trigger_stage": attempt.get(
                    "deadline_trigger_stage", ""),
                "remaining_budget_s": attempt.get("remaining_budget_s"),
                "candidate_audit": candidate,
            })
        attempts_document["requests"][name] = {
            "request": request,
            "request_sha256": record["request_sha256"],
            "initial_guesses": record["initial_guesses"],
            "attempts": attempt_rows,
            "attempt_count": len(attempt_rows),
        }

        solution = {
            "request": request,
            "request_sha256": record["request_sha256"],
            "status": result["status"],
            "reason": result["reason"],
            "detail": result["detail"],
            "solver_used": result["solver_used"],
            "wall_time_s": wall,
            "outer_wall_time_s": record["outer_wall_time_s"],
            "deadline_exhausted": result["deadline_exhausted"],
            "timeout_point": (
                deadline_summary.get("trigger_stage") or "NOT_TRIGGERED"),
            "deadline_summary": deadline_summary,
            "goal_heading_branch_rad": result["goal_heading_branch_rad"],
            "endpoint": endpoint,
            "endpoint_position_error_m": position_error,
            "endpoint_heading_error_rad": heading_error,
            "profile": record["profile"],
            "path_summary": record["path_summary"],
            "audit": audit,
            "audit_integration_budget": audit_budget,
            "attempt_count": len(attempt_rows),
            "attempts_in": "analysis/attempts.json",
        }
        solutions[name] = solution

        acceptance[name] = {
            "status": result["status"],
            "audit_passed": audit["passed"],
            "audit_reason": audit["reason"],
            "audit_check_count": audit["check_count"],
            "endpoint_position_error_m": position_error,
            "endpoint_position_limit_m": POSITION_TOL,
            "endpoint_heading_error_rad": heading_error,
            "endpoint_heading_limit_rad": HEADING_TOL,
            "rigidity_error_m": rigidity,
            "rigidity_limit_m": RIGIDITY_TOL,
            "position_budget_m": audit_integration["position_budget_m"],
            "position_budget_limit_m": POSITION_BUDGET,
            "angle_budget_rad": audit_integration["angle_budget_rad"],
            "angle_budget_limit_rad": ANGLE_BUDGET,
            "audit_integration_method": audit_budget["method"],
            "audit_position_budget_m": audit_budget["total_budget_m"],
            "audit_position_budget_limit_m": AUDIT_POSITION_THRESHOLD,
            "audit_angle_budget_rad":
                audit_integration["audit_angle_budget_rad"],
            "audit_budget_confirmed": audit_budget["confirmed"],
            "audit_total_subintervals": audit_budget["total_subintervals"],
            "audit_refinement_steps": audit_budget["refinement_steps"],
            "combined_position_budget_m":
                audit_integration["combined_position_budget_m"],
            "max_abs_C": max_c,
            "curvature_limit_1_per_m": curvature_limit,
            "max_static_wheel_angle_rad": max_wheel,
            "static_wheel_limit_rad": request["limits"]["steer_angle_max"],
            "c2_max_value_error": audit["c2"]["max_value_error"],
            "c2_max_slope_error": audit["c2"]["max_slope_error"],
            "c2_max_second_error": audit["c2"]["max_second_error"],
            "c2_max_abs_g": audit["c2"]["max_abs_g"],
            "whole_segment_count": len(segments),
            "whole_segment_leaves": [s["leaves"] for s in segments],
            "whole_segment_max_depth_reached": [
                s["max_depth_reached"] for s in segments],
            "all_segments_confirmed": all(s["passed"] for s in segments),
            "integration_independent_position_error_m":
                audit_integration["independent_position_error_m"],
            "integration_independent_heading_error_rad":
                audit_integration["independent_heading_error_rad"],
            "attempt_count": len(attempt_rows),
            "attempt_solvers": sorted(set(
                attempt["solver"] for attempt in attempts)),
            "attempt_statuses": sorted(set(
                attempt["status"] for attempt in attempts)),
            "iterations": [attempt["iterations"] for attempt in attempts],
            "wall_time_s": wall,
            "outer_wall_time_s": record["outer_wall_time_s"],
            "deadline_exhausted": result["deadline_exhausted"],
            "deadline_summary": deadline_summary,
        }

    attempts_document["success_attempt_count"] = success_attempt_count
    attempts_document["residual_nonnull_finite_count"] = \
        residual_nonnull_finite_count
    attempts_document["residual_null_count"] = residual_null_count
    attempts_document["residual_min"] = (
        min(residual_values) if residual_values else None)
    attempts_document["residual_max"] = (
        max(residual_values) if residual_values else None)
    if success_attempt_count != 48:
        all_ok = False
    if residual_nonnull_finite_count != success_attempt_count:
        all_ok = False
    if residual_null_count != 0:
        all_ok = False

    if not all_ok:
        raise SystemExit("one or more acceptance invariants failed")

    results_document = {
        "schema": "virtual_rigid_a.g2.results.v2",
        "stage": "G2",
        "run_id": RUN.name,
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "status": "PASS_PENDING_CODEX_RECHECK",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "scope": {
            "output_status": "GEOMETRY_FEASIBLE only",
            "timing": "NOT_EVALUATED",
            "dynamics": "NOT_EVALUATED",
            "map": "NOT_EVALUATED",
            "collision_certification": "NOT_CLAIMED",
            "ros_execution": "NOT_RUN",
            "hardware_validation": "NOT_CLAIMED",
            "executable_field": "NOT_PRESENT",
        },
        "frozen": {
            "manifest_path": "frozen/frozen_manifest.json",
            "manifest_sha256": sha256(FROZEN_MANIFEST),
            "open_requests_sha256":
                frozen["files"]["open_requests.json"]["frozen_sha256"],
            "existence_reference_sha256":
                frozen["files"]["existence_reference.json"]["frozen_sha256"],
            "planner_config_sha256": config_sha,
            "planner_config_is_default": True,
            "reference_profiles_passed_to_solve": False,
        },
        "config": config,
        "request_names": list(NAMES),
        "requests": {
            name: {
                "request_summary": results[name]["request"],
                "request_sha256": results[name]["request_sha256"],
            }
            for name in NAMES},
        "solutions": solutions,
        "attempts": {
            "path": "analysis/attempts.json",
            "sha256": None,
            "count": sum(len(results[name]["result"]["attempts"])
                         for name in NAMES),
            "success_attempt_count": success_attempt_count,
            "residual_nonnull_finite_count":
                residual_nonnull_finite_count,
            "residual_null_count": residual_null_count,
            "residual_min": attempts_document["residual_min"],
            "residual_max": attempts_document["residual_max"],
            "residual_definition":
                "NLP collocation/equality/signed-curvature diagnostic; "
                "not the independent whole-segment geometry audit",
        },
        "benchmark": {
            "path": "benchmark/summary.json",
            "sha256": sha256(SUMMARY_PATH),
            "results_path": "benchmark/results.json",
            "results_sha256": sha256(RESULT_PATH),
            "status": benchmark["status"],
            "checks": len(checks),
            "failed_checks": failed,
        },
        "audit_budget_crosscheck": {
            "path": "analysis/audit_budget_crosscheck.json",
            "sha256": sha256(crosscheck_path),
            "log": "logs/audit_budget_crosscheck.log",
            "all_checks_pass": True,
            "audit_total_budget_m":
                crosscheck["default_audit"]["integration"][
                    "audit_integration"]["total_budget_m"],
            "fixed_threshold_m": crosscheck["default_audit"]["integration"][
                "audit_integration"]["fixed_threshold_m"],
            "coarse_refinement_steps":
                crosscheck["coarse_refinement_audit"]["integration"][
                    "audit_integration"]["refinement_steps"],
            "independent_minus_quad_m":
                crosscheck["comparison"]["independent_minus_quad"][
                    "euclidean"],
            "returned_minus_independent_m":
                crosscheck["comparison"][
                    "returned_path_minus_independent"]["euclidean"],
            "resource_reason": crosscheck["resource_limited_audit"]["reason"],
            "resource_constraint": crosscheck["resource_limited_audit"][
                "location"].get("constraint"),
        },
        "auto_deadline_probe": {
            "path": "analysis/auto_deadline_probe.json",
            "sha256": sha256(probe_path),
            "log": "logs/auto_deadline_probe.log",
            "all_checks_pass": True,
            "schema": probe["schema"],
            "cases": {
                name: {
                    "checks_pass": record["checks_pass"],
                    "status": record["result"]["status"],
                    "deadline_exhausted":
                        record["result"]["deadline_exhausted"],
                    "trigger_stage":
                        record["result"]["deadline_summary"].get(
                            "trigger_stage"),
                    "timeout_solver_status":
                        record["result"]["deadline_summary"].get(
                            "timeout_solver_status"),
                    "attempts": [
                        [attempt["solver"], attempt["solver_status"],
                         attempt["status"]]
                        for attempt in record["result"]["attempts"]],
                }
                for name, record in probe["cases"].items()},
            "notes": probe["notes"],
        },
        "tests": {
            name: {
                "count": tests[name]["count"],
                "wall_time_s": tests[name]["wall_time_s"],
                "status": tests[name]["status"],
                "log": tests[name]["path"],
                "log_sha256": tests[name]["sha256"],
            }
            for name in EXPECTED_TEST_COUNTS},
        "negative": {
            "segment_interior_benchmark": benchmark["negative"][
                "segment_interior"],
            "counterexample": benchmark["negative"]["counterexample"],
            "coverage": coverage,
        },
        "acceptance": acceptance,
        "nonclaims": [
            "GEOMETRY_FEASIBLE only; no timing, dynamics, map, collision "
            "certification, ROS execution or hardware claim",
            "30 s is a soft total budget: IPOPT max_wall_time and the SLSQP "
            "iteration callback are checked, but library shutdown latency is "
            "recorded, not pre-empted; no hard-real-time guarantee is made",
            "the quadratic and clamped-spline references are independent "
            "oracles, never solver seeds or production inputs",
            "Simpson remainder arithmetic is certified analytically; libm/trig "
            "rounding remains a documented floating-point assumption, not a "
            "formal proof",
            "passing G2 tests and benchmark checks is not Codex acceptance",
        ],
    }
    attempts_path = RUN / "analysis" / "attempts.json"
    attempts_path.write_text(
        json.dumps(attempts_document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    results_document["attempts"]["sha256"] = sha256(attempts_path)
    (RUN / "results.json").write_text(
        json.dumps(results_document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    summary_document = {
        "schema": "virtual_rigid_a.g2.acceptance_summary.v2",
        "stage": "G2",
        "run_id": RUN.name,
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "status": "PASS_PENDING_CODEX_RECHECK",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "frozen_config": {
            "values": config,
            "sha256_canonical": config_sha,
            "equals_PlannerConfig_default": True,
            "max_seconds": config["max_seconds"],
            "max_initials": config["max_initials"],
            "max_iterations": config["max_iterations"],
            "intervals": config["intervals"],
            "solver": config["solver"],
        },
        "per_request": acceptance,
        "all_requests_pass": all_ok,
        "all_requests_status": sorted(set(
            acceptance[name]["status"] for name in NAMES)),
        "all_audits_passed": all(acceptance[name]["audit_passed"]
                                 for name in NAMES),
        "constraint_residual_summary": {
            "success_attempt_count": success_attempt_count,
            "residual_nonnull_finite_count":
                residual_nonnull_finite_count,
            "residual_null_count": residual_null_count,
            "residual_min": attempts_document["residual_min"],
            "residual_max": attempts_document["residual_max"],
            "definition": (
                "NLP collocation/equality/signed-curvature diagnostic; "
                "not a replacement for the independent whole-segment audit"),
            "source": "analysis/attempts.json",
        },
        "required_request_count": len(NAMES),
        "required_requests": list(NAMES),
        "deadline": {
            "soft_budget_s": config["max_seconds"],
            "max_observed_solve_wall_time_s": max_wall,
            "timeout_point": "NOT_TRIGGERED",
            "any_deadline_exhausted": any(
                acceptance[name]["deadline_exhausted"] for name in NAMES),
            "ipopt_control": "IPOPT option max_wall_time",
            "slsqp_control": "Python iteration callback raising _SoftDeadline",
            "hard_real_time_guarantee": False,
            "semantics": (
                "remaining budget is checked at initial and attempt "
                "boundaries; IPOPT receives max_wall_time and SLSQP checks "
                "the deadline in its iteration callback. A timed-out iterate "
                "is never audited or published. The solver library may "
                "return after its own shutdown overhead, so no hard "
                "real-time guarantee is claimed"),
        },
        "maximums": {
            "endpoint_position_error_m": max(
                acceptance[name]["endpoint_position_error_m"]
                for name in NAMES),
            "endpoint_heading_error_rad": max(
                acceptance[name]["endpoint_heading_error_rad"]
                for name in NAMES),
            "rigidity_error_m": max(
                acceptance[name]["rigidity_error_m"] for name in NAMES),
            "position_budget_m": max(
                acceptance[name]["position_budget_m"] for name in NAMES),
            "angle_budget_rad": max(
                acceptance[name]["angle_budget_rad"] for name in NAMES),
            "audit_position_budget_m": max(
                acceptance[name]["audit_position_budget_m"]
                for name in NAMES),
            "audit_angle_budget_rad": max(
                acceptance[name]["audit_angle_budget_rad"]
                for name in NAMES),
            "combined_position_budget_m": max(
                acceptance[name]["combined_position_budget_m"]
                for name in NAMES),
            "max_abs_C": max(acceptance[name]["max_abs_C"]
                             for name in NAMES),
            "max_static_wheel_angle_rad": max(
                acceptance[name]["max_static_wheel_angle_rad"]
                for name in NAMES),
        },
        "limits": {
            "endpoint_position_m": POSITION_TOL,
            "endpoint_heading_rad": HEADING_TOL,
            "endpoint_heading_deg": 0.2,
            "rigidity_m": RIGIDITY_TOL,
            "position_budget_m": POSITION_BUDGET,
            "angle_budget_rad": ANGLE_BUDGET,
            "audit_position_budget_m": AUDIT_POSITION_THRESHOLD,
            "audit_integration_method": AUDIT_METHOD,
            "per_request_curvature_and_wheel": per_request_limits,
        },
        "auto_deadline_probe": {
            "path": "analysis/auto_deadline_probe.json",
            "sha256": sha256(probe_path),
            "all_checks_pass": True,
            "cases": {
                name: {
                    "checks_pass": record["checks_pass"],
                    "status": record["result"]["status"],
                    "trigger_stage":
                        record["result"]["deadline_summary"].get(
                            "trigger_stage"),
                    "timeout_solver_status":
                        record["result"]["deadline_summary"].get(
                            "timeout_solver_status"),
                }
                for name, record in probe["cases"].items()},
        },
        "tests": {
            name: {
                "count": tests[name]["count"],
                "wall_time_s": tests[name]["wall_time_s"],
                "status": tests[name]["status"],
            }
            for name in EXPECTED_TEST_COUNTS},
        "benchmark": {
            "status": benchmark["status"],
            "checks": len(checks),
            "failed": failed,
            "checks_sha256": canonical_sha(checks),
        },
        "audit_budget_crosscheck": results_document[
            "audit_budget_crosscheck"],
        "negative_fixture_coverage": coverage,
        "segment_interior_fixture": {
            "knots_s_k_eta": [[0.0, 0.0, 0.0], [0.1, 0.0, 10.0]],
            "hermite_in_q_where_q=s/h": {
                "h_m": 0.1, "c0": 0.0, "c1": 0.0, "c2": -1.0, "c3": 1.0,
                "k_q": "-q^2+q^3", "eta_q": "(-2q+3q^2)/h"},
            "benchmark_first_failure": benchmark["negative"][
                "segment_interior"]["first_failure"],
            "benchmark_reason": benchmark["negative"][
                "segment_interior"]["reason"],
        },
        "wheel_model": {
            "definition": (
                "ideal static wheel angle: xw=+/-wheelbase/2, "
                "yw=+/-track/2, m=1-yw*C, n=xw*C, delta=atan2(n,m); "
                "max |delta| over four wheels"),
            "source": "fwis_v3_optimizer/retiming.py wheel_configuration",
            "audit_uses_interval_endpoints": True,
            "gross_ackermann_formula_used": False,
        },
        "nonclaims": results_document["nonclaims"],
    }
    (RUN / "analysis" / "acceptance_summary.json").write_text(
        json.dumps(summary_document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "ASSEMBLED",
        "run_id": RUN.name,
        "results_sha256": sha256(RUN / "results.json"),
        "attempts_sha256": sha256(RUN / "analysis" / "attempts.json"),
        "acceptance_summary_sha256": sha256(
            RUN / "analysis" / "acceptance_summary.json"),
        "attempt_count": results_document["attempts"]["count"],
        "success_attempt_count": success_attempt_count,
        "residual_nonnull_finite_count":
            residual_nonnull_finite_count,
        "residual_null_count": residual_null_count,
        "residual_min": attempts_document["residual_min"],
        "residual_max": attempts_document["residual_max"],
        "max_wall_time_s": max_wall,
        "maximums": summary_document["maximums"],
        "tests": {name: tests[name]["count"] for name in tests},
        "benchmark_checks": len(checks),
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()