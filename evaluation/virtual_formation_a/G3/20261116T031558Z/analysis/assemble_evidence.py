"""Assemble the G3 result summary from the frozen evidence files.

Read-only for source and benchmark/log artifacts.  Every status is derived from
the recorded benchmark, test logs and regression records; this script never
writes Codex acceptance.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path
import re

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
EXPECTED_TEST_COUNTS = {"timing": 18, "trajectory": 23, "audit": 51}
EXPECTED_BENCHMARK_CHECKS = 151
EXPECTED_CASES = [
    "E1", "E2", "E3", "E4", "E5", "E6", "NO_MOTION_HOLD",
    "STRAIGHT_LINE", "STRAIGHT_POLICY"]
MOTION_CASES = [name for name in EXPECTED_CASES if name != "NO_MOTION_HOLD"]
DIFFERENTIAL_TOLERANCES = {
    "velocity": 1e-5, "angular_rate": 1e-5,
    "acceleration": 1e-4, "angular_accel": 1e-4,
    "jerk": 1e-2, "angular_jerk": 1e-2,
}
INVERSE_TOLERANCE_S = 1e-9
EXPECTED_TAMPER = {
    "timing_T": "MODEL_TIMING_INCONSISTENT",
    "extra_field": "UNSUPPORTED_MODEL_FIELD",
    "self_digest": "UNSUPPORTED_MODEL_FIELD",
}
FORBIDDEN_MODEL_FIELDS = (
    "model_sha256", "dynamic_audit", "dynamic_audit_passed",
    "executable", "collision_checked")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def parse_test_log(name):
    text = (RUN / "logs" / (name + ".log")).read_text(encoding="utf-8")
    match = re.search(r"Ran (\d+) tests? in ([0-9.]+)s", text)
    if match is None:
        raise SystemExit("{} log has no unittest summary".format(name))
    count = int(match.group(1))
    if count != EXPECTED_TEST_COUNTS[name]:
        raise SystemExit("{} test count {} != {}".format(
            name, count, EXPECTED_TEST_COUNTS[name]))
    if not text.rstrip().splitlines()[-1].strip() == "OK":
        raise SystemExit("{} log did not end with OK".format(name))
    return {
        "count": count,
        "wall_time_s": float(match.group(2)),
        "status": "OK",
        "log_sha256": sha256(RUN / "logs" / (name + ".log")),
        "log": "logs/{}.log".format(name),
    }


def model_case(name, record):
    timing = record["timing"]
    bounds = timing["bounds_summary"]
    audit = record["dynamic_audit"]
    document = json.loads(record["model_json"])
    if record["model_sha256"] != hashlib.sha256(
            record["model_json"].encode("utf-8")).hexdigest():
        raise SystemExit("{} model digest is not the canonical model".format(
            name))
    if document.get("request_sha256") != record["request_sha256"]:
        raise SystemExit("{} model request digest changed".format(name))
    for forbidden in FORBIDDEN_MODEL_FIELDS:
        if forbidden in document:
            raise SystemExit("{} model contains {!r}".format(
                name, forbidden))
    for key in ("backend", "model_version", "path_model_version",
                "request_inputs", "request_sha256", "path", "timing",
                "numerics", "planning_context"):
        if key not in document:
            raise SystemExit("{} model missing {!r}".format(name, key))
    if document["timing"]["kind"] != (
            "hold" if name == "NO_MOTION_HOLD" else "motion"):
        raise SystemExit("{} model kind changed".format(name))
    if document["timing"]["T_s"] != timing["T_s"]:
        raise SystemExit("{} model T differs from timing".format(name))
    if record["roundtrip"].get("equal_json") is not True or \
            record["roundtrip"].get("equal_digest") is not True:
        raise SystemExit("{} roundtrip did not restore the model".format(name))
    if record["sample"].get("authoritative") is not False:
        raise SystemExit("{} sample claimed authority".format(name))
    result = {
        "name": name,
        "request_sha256": record["request_sha256"],
        "model_sha256": record["model_sha256"],
        "model_json_location": (
            "benchmark/results.json:{} .model_json".format(name)),
        "timing_status": timing["status"],
        "T_s": timing["T_s"],
        "roundtrip": record["roundtrip"],
        "sample": record["sample"],
    }
    if name == "NO_MOTION_HOLD":
        if timing["status"] != "NO_MOTION" or timing["T_s"] != 0.0:
            raise SystemExit("hold case is not an exact hold")
        if record["sample"].get("times") != [0.0]:
            raise SystemExit("hold sample is not the single t=0 sample")
        result["S_m"] = 0.0
        result["selection"] = None
    else:
        if timing["status"] != "DYNAMIC_FEASIBLE":
            raise SystemExit("{} timing status {}".format(
                name, timing["status"]))
        selection = bounds
        metrics = selection["metrics_at_T0_s"]
        recomputed = max(
            metrics[metric]["required_duration_s"]
            for metric in metrics)
        if selection["required_duration_s"] != recomputed:
            raise SystemExit("{} selection required is not its metric max"
                             .format(name))
        if abs(selection["selected_duration_s"]
                - selection["required_duration_s"]
                * selection["safety_factor"]) > 1e-9:
            raise SystemExit("{} selected duration is inconsistent".format(
                name))
        if timing["T_s"] != selection["selected_duration_s"]:
            raise SystemExit("{} T differs from the selection".format(name))
        worst = selection["worst"]
        worst_record = metrics[worst["quantity"]]
        if worst_record["required_duration_s"] != \
                selection["required_duration_s"]:
            raise SystemExit("{} worst is not the binding metric".format(name))
        if document["timing"]["selection"] != _canonical_selection(selection):
            raise SystemExit("{} model selection differs from timing".format(
                name))
        result.update({
            "S_m": document["timing"]["S"],
            "selection": {
                "required_duration_s": selection["required_duration_s"],
                "selected_duration_s": selection["selected_duration_s"],
                "safety_factor": selection["safety_factor"],
                "rescalings": selection["rescalings"],
                "T0_s": selection["T0_s"],
                "interval_subdivisions_per_segment":
                    selection["interval_subdivisions_per_segment"],
                "worst": worst,
                "worst_metric": worst_record,
                "wheel_angle_bound_rad":
                    selection["wheel_angle_bound_rad"],
                "wheel_angle_limit_rad":
                    selection["wheel_angle_limit_rad"],
            },
        })
    if audit is None or audit.get("passed") is not True:
        raise SystemExit("{} dynamic audit did not pass".format(name))
    if audit.get("scope") != "DYNAMICS":
        raise SystemExit("{} audit scope is not DYNAMICS".format(name))
    metrics = audit["metrics"]
    for key in ("collision_checked", "executable", "map_used", "ros_used"):
        if metrics.get(key) is not False:
            raise SystemExit("{} audit claims {}".format(name, key))
    if name == "NO_MOTION_HOLD":
        hold = metrics.get("hold")
        if not isinstance(hold, dict) or \
                hold.get("hold_model") != "explicit_zero_duration":
            raise SystemExit("hold audit did not use the hold model")
        if hold.get("inverse_division_free") is not True or \
                hold.get("sample_times") != [0.0] or \
                hold.get("max_state_error") != 0.0:
            raise SystemExit("hold audit violated the exact-hold contract")
        if metrics.get("kind") != "hold" or metrics.get("motion") is not False:
            raise SystemExit("hold audit is not marked hold")
        for key in ("differential", "time_inverse"):
            if key in metrics:
                raise SystemExit("hold audit used motion inverse formulas")
        result["audit"] = {
            "check_count": audit["check_count"],
            "hold": hold,
            "whole_domain_certificate": False,
            "certificate_note": (
                "not applicable: NO_MOTION uses the exact hold model "
                "S=T=0 with one t=0 sample, not the motion time law"),
            "collision_checked": False,
            "executable": False,
            "map_used": False,
            "ros_used": False,
        }
        return result
    if metrics.get("whole_domain_certificate") is not True:
        raise SystemExit("{} whole-domain certificate missing".format(name))
    nodes = metrics["nodes"]
    inverse = metrics["time_inverse"]
    differential = metrics["differential"]
    if nodes["internal_node_count"] != len(nodes["records"]):
        raise SystemExit("{} node record count mismatch".format(name))
    if inverse["sample_count"] != 100:
        raise SystemExit("{} inverse sample count is not 100".format(name))
    if inverse["max_error_s"] > INVERSE_TOLERANCE_S:
        raise SystemExit("{} inverse error exceeds tolerance".format(name))
    for metric, tolerance in DIFFERENTIAL_TOLERANCES.items():
        if differential["max_errors"][metric] > tolerance:
            raise SystemExit("{} differential {} exceeds tolerance".format(
                name, metric))
    if differential["checked_sample_count"] <= 0 or \
            differential["skipped_sample_count"] < 0:
        raise SystemExit("{} differential sample counts are invalid".format(
            name))
    pointwise = metrics.get("pointwise_interval_checks")
    if not isinstance(pointwise, dict) or \
            pointwise.get("leaf_count") != metrics["leaves"]:
        raise SystemExit("{} pointwise coverage is not every leaf".format(name))
    if pointwise.get("vehicle_count") != 3 * pointwise["leaf_count"]:
        raise SystemExit("{} pointwise coverage misses a vehicle".format(name))
    for key in ("max_jerk_ratio", "max_component_ratio",
                "max_p_sss_ratio"):
        value = pointwise.get(key)
        if not isinstance(value, (int, float)) or not 0.0 <= value <= 1.0:
            raise SystemExit("{} pointwise ratio {}={!r} is invalid".format(
                name, key, value))
    worst = pointwise.get("worst")
    if not isinstance(worst, dict) or worst.get("actual") is None or \
            worst.get("bound") is None or \
            worst["actual"] > worst["bound"]:
        raise SystemExit("{} pointwise worst record is not contained".format(
            name))
    node_rows = []
    for index, node in enumerate(nodes["records"], start=1):
        node_rows.append({
            "node": index,
            "s": node["s"],
            "t": node["t"],
            "tau": node["tau"],
            "left_region": node["left_region"],
            "right_region": node["right_region"],
            "auto_region": node["auto_region"],
            "w": node["w"],
            "jump_max": node["jump_max"],
            "max_continuity_error": node["max_continuity_error"],
            "inverse_error_s": node["inverse_error_s"],
            "left_jerk_vector_location":
                "benchmark/results.json:{} .dynamic_audit.metrics."
                "nodes.records[{}].left_jerk".format(name, index - 1),
            "right_jerk_vector_location":
                "benchmark/results.json:{} .dynamic_audit.metrics."
                "nodes.records[{}].right_jerk".format(name, index - 1),
            "pointwise_interval": node["pointwise_interval"],
        })
        for side in ("left", "right"):
            coverage = node["pointwise_interval"][side]
            for key in ("jerk_ratio", "component_ratio", "p_sss_ratio"):
                if not 0.0 <= coverage[key] <= 1.0:
                    raise SystemExit(
                        "{} node {} {} {} is not contained".format(
                            name, index, side, key))
            if coverage["worst"]["actual"] > coverage["worst"]["bound"]:
                raise SystemExit(
                    "{} node {} {} worst is not contained".format(
                        name, index, side))
    result["audit"] = {
        "check_count": audit["check_count"],
        "leaves": metrics["leaves"],
        "max_utilization": metrics["max_utilization"],
        "pointwise_interval_checks": pointwise,
        "whole_domain_certificate": True,
        "collision_checked": False,
        "executable": False,
        "map_used": False,
        "ros_used": False,
        "nodes": {
            "internal_node_count": nodes["internal_node_count"],
            "max_continuity_error": nodes["max_continuity_error"],
            "max_state_error": nodes["max_state_error"],
            "max_inverse_error_s": nodes["max_inverse_error_s"],
            "max_jump_magnitude": nodes["max_jump_magnitude"],
            "records": node_rows,
        },
        "time_inverse": {
            "sample_count": inverse["sample_count"],
            "max_error_s": inverse["max_error_s"],
            "max_independent_error_s":
                inverse["max_independent_error_s"],
            "tolerance_s": inverse["tolerance_s"],
        },
        "differential": {
            "oracle": differential["oracle"],
            "checked_sample_count":
                differential["checked_sample_count"],
            "skipped_sample_count":
                differential["skipped_sample_count"],
            "max_errors": differential["max_errors"],
            "tolerances": differential["tolerances"],
        },
    }
    short = record.get("short_duration")
    if short is None or short.get("passed") is not False or \
            short.get("reason") != "DYNAMIC_LIMIT":
        raise SystemExit("{} short-duration audit was not rejected".format(
            name))
    failure = short["first_failure"]
    result["short_duration"] = {
        "rejected": True,
        "reason": short["reason"],
        "check_count": short["check_count"],
        "first_failure": {
            "quantity": failure["quantity"],
            "vehicle_id": failure["vehicle_id"],
            "wheel_index": failure["wheel_index"],
            "value": failure["value"],
            "limit": failure["limit"],
        },
    }
    tamper = record.get("tamper")
    if tamper != EXPECTED_TAMPER:
        raise SystemExit("{} tamper reasons changed: {}".format(name, tamper))
    result["tamper"] = tamper
    return result


def _canonical_selection(selection):
    return {
        key: selection[key]
        for key in (
            "required_duration_s", "selected_duration_s", "safety_factor",
            "worst", "metrics_at_T0_s", "wheel_angle_bound_rad",
            "wheel_angle_limit_rad", "interval_subdivisions_per_segment",
            "T0_s", "rescalings")}


def main():
    tests = {name: parse_test_log(name)
             for name in EXPECTED_TEST_COUNTS}
    benchmark = load(RUN / "benchmark" / "summary.json")
    records = load(RUN / "benchmark" / "results.json")
    if benchmark.get("status") != "PASS":
        raise SystemExit("benchmark status is not PASS")
    if benchmark.get("codex_acceptance") != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("benchmark claims Codex acceptance")
    if len(benchmark.get("checks", [])) != EXPECTED_BENCHMARK_CHECKS:
        raise SystemExit("benchmark check count changed")
    if any(not check["pass"] for check in benchmark["checks"]):
        raise SystemExit("benchmark has a failed check")
    if sorted(records) != sorted(EXPECTED_CASES):
        raise SystemExit("benchmark case set changed")

    cases = [model_case(name, records[name]) for name in EXPECTED_CASES]
    by_name = {case["name"]: case for case in cases}
    motion = [by_name[name] for name in MOTION_CASES]
    aggregate = {
        "benchmark_check_count": len(benchmark["checks"]),
        "benchmark_failed_checks": sum(
            not check["pass"] for check in benchmark["checks"]),
        "dynamic_feasible_cases": len(motion),
        "no_motion_cases": 1,
        "max_required_duration_s": max(
            case["selection"]["required_duration_s"] for case in motion),
        "max_selected_duration_s": max(
            case["selection"]["selected_duration_s"] for case in motion),
        "total_rescalings": sum(
            case["selection"]["rescalings"] for case in motion),
        "max_differential_errors": {
            metric: max(
                case["audit"]["differential"]["max_errors"][metric]
                for case in motion)
            for metric in DIFFERENTIAL_TOLERANCES},
        "differential_tolerances": dict(DIFFERENTIAL_TOLERANCES),
        "max_time_inverse_error_s": max(
            case["audit"]["time_inverse"]["max_error_s"]
            for case in motion),
        "max_independent_time_inverse_error_s": max(
            case["audit"]["time_inverse"]["max_independent_error_s"]
            for case in motion),
        "time_inverse_tolerance_s": INVERSE_TOLERANCE_S,
        "inverse_samples_per_case": 100,
        "max_node_continuity_error": max(
            case["audit"]["nodes"]["max_continuity_error"]
            for case in motion),
        "max_node_state_error": max(
            case["audit"]["nodes"]["max_state_error"]
            for case in motion),
        "max_node_jump_magnitude": max(
            case["audit"]["nodes"]["max_jump_magnitude"]
            for case in motion),
        "max_actual_utilization": max(
            max(case["audit"]["max_utilization"].values())
            for case in motion),
        "pointwise_leaf_count": sum(
            case["audit"]["pointwise_interval_checks"]["leaf_count"]
            for case in motion),
        "pointwise_vehicle_count": sum(
            case["audit"]["pointwise_interval_checks"]["vehicle_count"]
            for case in motion),
        "max_pointwise_jerk_ratio": max(
            case["audit"]["pointwise_interval_checks"]["max_jerk_ratio"]
            for case in motion),
        "max_pointwise_component_ratio": max(
            case["audit"]["pointwise_interval_checks"]
            ["max_component_ratio"] for case in motion),
        "max_pointwise_p_sss_ratio": max(
            case["audit"]["pointwise_interval_checks"]["max_p_sss_ratio"]
            for case in motion),
        "short_duration_rejections": sum(
            case["short_duration"]["rejected"] for case in motion),
        "tamper_rejections": sum(
            all(case["tamper"][key] == value
                for key, value in EXPECTED_TAMPER.items())
            for case in motion),
        "all_dynamic_audits_passed": all(
            case["audit"]["check_count"] > 0 for case in cases),
    }
    for metric, value in aggregate["max_differential_errors"].items():
        if value > DIFFERENTIAL_TOLERANCES[metric]:
            raise SystemExit("aggregate differential {} exceeds tolerance"
                             .format(metric))
    if aggregate["max_time_inverse_error_s"] > INVERSE_TOLERANCE_S:
        raise SystemExit("aggregate inverse error exceeds tolerance")
    for key in ("max_pointwise_jerk_ratio",
                "max_pointwise_component_ratio",
                "max_pointwise_p_sss_ratio"):
        if aggregate[key] > 1.0:
            raise SystemExit("aggregate {} exceeds 1".format(key))

    legacy = load(RUN / "legacy" / "regression_results.json")
    legacy_cases = {
        row["case_id"]: {
            "status": row["status"],
            "returncode": row["returncode"],
            "wall_s": row["wall_s"],
            "stderr_log": row["stderr_log"],
        }
        for row in legacy["records"]}
    if legacy["status"] != "FAIL":
        raise SystemExit("legacy regression status is not the observed FAIL")
    if sorted(legacy_cases) != sorted([
            "timing_continuous", "timing_formation", "fr02_a_timing",
            "fr02_a_trajectory", "fr02_b_quality"]):
        raise SystemExit("legacy regression case set changed")
    if legacy_cases["timing_continuous"]["status"] != "FAIL" or \
            legacy_cases["timing_continuous"]["returncode"] != 1:
        raise SystemExit("legacy timing_continuous failure changed")
    for name in ("timing_formation", "fr02_a_timing",
                 "fr02_a_trajectory", "fr02_b_quality"):
        if legacy_cases[name]["status"] != "PASS" or \
                legacy_cases[name]["returncode"] != 0:
            raise SystemExit("legacy case {} is not PASS".format(name))
    unmodified_gate = (RUN / "logs" /
                       "legacy_unmodified_gate.log").read_text(
                           encoding="utf-8").strip()
    if "UNKNOWN_WORKSPACE_MODIFICATIONS" not in unmodified_gate:
        raise SystemExit("unmodified-gate failure was not recorded")
    probe = (RUN / "logs" / "legacy_baseline_probe.log").read_text(
        encoding="utf-8")
    if "KeyError: 'dynamic_peaks'" not in probe or "probe_exit=1" not in probe:
        raise SystemExit("pristine-HEAD probe did not reproduce the failure")
    if probe.count(
            "b7a36fd70c5d2bc5535334836d958a10afa10109f0c7cd2b226745179c7eee0c") \
            < 1:
        raise SystemExit("pristine probe test file hash missing")
    if probe.count(
            "2d1e9b182028e63d5fa9b25566ea0b9ab0cccb4ea7d3921ccfe651f3186ea8ef") \
            < 1:
        raise SystemExit("pristine probe source hash missing")

    numerics = load(RUN / "analysis" / "g3_rework_numerics.json")
    degenerate = numerics["degenerate_counterexample"]
    for name in ("timing", "audit"):
        record = degenerate[name]
        if not (record["covers_expected_norm"]
                and all(record["covers_expected_components"])):
            raise SystemExit(
                "{} degenerate interval misses the counterexample".format(name))
        if record["jerk"] < degenerate["expected_p_sss_norm"]:
            raise SystemExit("{} degenerate jerk under-estimates".format(name))
    narrow = numerics["narrow_interval_coverage"]
    if narrow["failure_count"] != 0 or narrow["case_count"] < 24:
        raise SystemExit("narrow interval component coverage failed")
    positive = numerics["positive_paths"]
    if len(positive) != 2 or not all(
            record["all_contained"] for record in positive):
        raise SystemExit("positive real-path pointwise coverage failed")
    signs = set()
    for record in positive:
        signs.update(tuple(value) for value in
                     record["front_vehicle"]["sign_combinations"])
        if record["front_vehicle"]["nonzero_k_times_eta"] <= 0:
            raise SystemExit("front vehicle has no nonzero k*eta sample")
    if signs != {(-1, -1), (-1, 1), (1, -1), (1, 1)}:
        raise SystemExit("positive paths do not cover all four sign pairs")
    negative = numerics["jerk_overlimit_negative"]["shortened_plan"]
    if negative["audit_passed"] or negative["audit_reason"] != "DYNAMIC_LIMIT":
        raise SystemExit("shortened jerk negative was not rejected")
    if negative["scanned_maximum_away_from_endpoints"]["jerk"] <= \
            numerics["jerk_overlimit_negative"]["limits"]["jerk_max"]:
        raise SystemExit("no real interior jerk over-limit was recorded")
    fail_closed = numerics["broad_interval_fail_closed"]
    if fail_closed["coarse"]["reason"] != "BOUND_UNCONFIRMED" or \
            not fail_closed["refined"]["passed"]:
        raise SystemExit("broad-interval fail-closed evidence changed")

    protection = load(RUN / "analysis" /
                      "protection_diff_vs_g2_accepted.json")
    if protection.get("codex_acceptance") != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("protection diff claims Codex acceptance")
    if protection["authorization"][
            "only_authorized_changes"] is not True:
        raise SystemExit("protection diff is not exactly authorized")

    document = {
        "schema": "virtual_rigid_a.g3.rework.results.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "stage": "G3",
        "status": "PASS_PENDING_CODEX_RECHECK",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "scope": {
            "output_status": (
                "DYNAMIC_FEASIBLE only for plans that passed the independent "
                "raw-profile whole-domain dynamic audit"),
            "common_time": "PROVEN_FOR_TESTED_PATHS",
            "three_vehicle_dynamics": "AUDITED_WHOLE_DOMAIN",
            "timing_and_dynamics_proven": True,
            "collision_checked": False,
            "executable": False,
            "map_used": False,
            "ros_used": False,
            "gazebo_used": False,
            "hardware_used": False,
            "time_optimal": False,
            "g4_entered": False,
        },
        "tests": tests,
        "benchmark": {
            "status": benchmark["status"],
            "check_count": len(benchmark["checks"]),
            "failed": [check for check in benchmark["checks"]
                       if not check["pass"]],
            "codex_acceptance": benchmark["codex_acceptance"],
            "nonclaims": benchmark.get("nonclaims"),
        },
        "model_cases": cases,
        "aggregate": aggregate,
        "legacy": {
            "unmodified_g0_gate": {
                "status": "BLOCKED",
                "exit_code": 1,
                "detail": unmodified_gate,
                "log": "logs/legacy_unmodified_gate.log",
            },
            "disclosed_wrapper": {
                "status": legacy["status"],
                "script": "baseline_runner_rework_docs_and_fixtures.py",
                "scope": (
                    "adds only the prior-stage rework documents, the G3 "
                    "rework task document and G2 fixtures that accepted "
                    "G1/G2/G3 explicitly authorized to the G0 in-memory "
                    "allowlists; edits no workspace file"),
                "cases": legacy_cases,
                "log": "logs/legacy.log",
            },
            "pre_existing_failure_proof": {
                "case": "timing_continuous",
                "failure": (
                    "test_short_translation_has_finite_acceleration_and_zero_"
                    "end_speed KeyError: 'dynamic_peaks'"),
                "failed_test_file_sha256":
                    "b7a36fd70c5d2bc5535334836d958a10afa10109f0c7cd2b226745179c7eee0c",
                "failed_source_file_sha256":
                    "2d1e9b182028e63d5fa9b25566ea0b9ab0cccb4ea7d3921ccfe651f3186ea8ef",
                "reproduced_from": "git archive HEAD pristine export",
                "export_root": "/tmp/g3_rework_baseline_probe_20261116",
                "exit_code": 1,
                "log": "logs/legacy_baseline_probe.log",
                "conclusion": (
                    "pre-existing baseline test staleness, not a G3 source "
                    "effect; old tests were not modified or relaxed"),
            },
        },
        "protection": {
            "g2_accepted_tree_sha256":
                protection["g2_evidence"]["protected_tree_sha256"],
            "g3_full_protected_tree_sha256":
                protection["current"]["full_protected_tree_sha256"],
            "added": [row["path"] for row in
                      protection["added_vs_g2_accepted"]],
            "changed": [row["path"] for row in
                        protection["changed_vs_g2_accepted"]],
            "removed": protection["removed_vs_g2_accepted"],
            "only_authorized_changes":
                protection["authorization"][
                    "only_authorized_changes"],
            "diff_location":
                "analysis/protection_diff_vs_g2_accepted.json",
            "diff_sha256": sha256(
                RUN / "analysis" /
                "protection_diff_vs_g2_accepted.json"),
        },
        "interval_coverage": {
            "degenerate_counterexample": {
                "expected_p_sss_norm":
                    degenerate["expected_p_sss_norm"],
                "timing_jerk": degenerate["timing"]["jerk"],
                "audit_jerk": degenerate["audit"]["jerk"],
            },
            "narrow_interval_case_count": narrow["case_count"],
            "narrow_interval_failure_count": narrow["failure_count"],
            "positive_paths": [
                {
                    "name": record["name"],
                    "T_s": record["timing"]["T_s"],
                    "interior_samples": record["sample_count"],
                    "node_side_samples": record["node_sample_count"],
                    "front_nonzero_k_eta_samples":
                        record["front_vehicle"]["nonzero_k_times_eta"],
                    "front_sign_pairs":
                        record["front_vehicle"]["sign_combinations"],
                    "max_front_jerk_ratio":
                        record["front_vehicle"]["max_jerk_ratio"],
                    "all_contained": record["all_contained"],
                }
                for record in positive],
            "jerk_overlimit_negative": {
                "limit": numerics["jerk_overlimit_negative"]["limits"],
                "audit_reason": negative["audit_reason"],
                "first_failure": negative["first_failure"],
                "interior_scan_maximum":
                    negative["scanned_maximum_away_from_endpoints"],
                "interior_overlimit_samples":
                    negative["scan_over_limit_count_away_from_endpoints"],
            },
            "broad_interval_fail_closed": fail_closed,
            "numerics_location": "analysis/g3_rework_numerics.json",
            "numerics_sha256": sha256(
                RUN / "analysis" / "g3_rework_numerics.json"),
        },
        "nonclaims": [
            "DYNAMIC_FEASIBLE is a common-time whole-domain dynamic audit "
            "result, not collision freedom, executability, time optimality, "
            "map feasibility, ROS/Gazebo validation or hardware acceptance",
            "collision_checked=false, executable=false, map_used=false, "
            "ros_used=false; no map, ROS, Gazebo or hardware run occurred",
            "the whole-domain jerk certificate is a continuous interval proof in exact "
            "arithmetic followed by outward floating-point rounding; the "
            "pointwise independent formula and the 20001-point scan are "
            "coverage witnesses, not substitutes for that proof",
            "the NO_MOTION case is an exact hold with S=T=0 and one t=0 "
            "sample; no division or inverse formula is used there",
            "the legacy regression overall FAIL is reported truthfully; four "
            "cases pass and timing_continuous has a pre-existing baseline "
            "failure reproduced from a pristine HEAD export",
            "no Git commit, push, branch creation or other worktree operation "
            "occurred; Codex acceptance is NOT_RUN_NOT_CLAIMED",
        ],
    }
    target = RUN / "results.json"
    target.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "run_id": RUN.name,
        "results": str(target.relative_to(REPO_ROOT)),
        "results_sha256": sha256(target),
        "cases": len(cases),
        "motion_cases": len(motion),
        "benchmark_checks": aggregate["benchmark_check_count"],
        "dynamic_audits_passed": aggregate["all_dynamic_audits_passed"],
        "short_duration_rejections":
            aggregate["short_duration_rejections"],
        "max_differential_errors": aggregate["max_differential_errors"],
        "max_time_inverse_error_s":
            aggregate["max_time_inverse_error_s"],
        "pointwise_leaf_count": aggregate["pointwise_leaf_count"],
        "max_pointwise_jerk_ratio":
            aggregate["max_pointwise_jerk_ratio"],
        "max_pointwise_component_ratio":
            aggregate["max_pointwise_component_ratio"],
        "max_pointwise_p_sss_ratio":
            aggregate["max_pointwise_p_sss_ratio"],
        "legacy_status": legacy["status"],
        "protection_only_authorized":
            document["protection"]["only_authorized_changes"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()