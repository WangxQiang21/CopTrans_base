#!/usr/bin/env python3
"""Check R12 frozen acceptance artifacts, keeping warnings separate from gates.

This checks saved independent audits and exact input/source identities; it does
not turn missing history, a native legacy clock, or a display box into a safety
certificate. The prior RViz visual signoff remains a separate human review.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from audit_corridor_display import audit as audit_display

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/"relust/R12_planning_reliability_20261012"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect_run(path, expected):
    path = path.resolve()
    run, probe, case = [read(path/name) for name in
                        ("run.json", "probe_result.json", "case.json")]
    params = yaml.safe_load((path/"params.yaml").read_text())
    defaults = yaml.safe_load((ROOT/"catkin_ws/src/fwis_v3_demo/config/params.yaml").read_text())
    report = probe["quality_report"]
    selected, timing = report["selected"], report["selected_timing"]
    checks = {}
    checks["exact_frozen_input"] = all(case[k] == expected[k] for k in
        ("map_dir", "map_scale", "start", "goal", "xy_tol", "yaw_tol"))
    checks["body_contract"] = (params["vehicle"]["length"] == .738 and
        params["vehicle"]["width"] == .5 and params["collision"]["margin"] == .04)
    checks["restricted_profile"] = (params["quality"]["limits"] == defaults["quality"]["limits"] and
                                     params["vehicle"] == defaults["vehicle"])
    checks["direction_contract"] = (params["planning"]["allow_shunting"] is False and
        probe["independent_direction"]["shunting_allowed_by_config"] is False)
    checks["checker_contract"] = run["checker"] == {
        "sampling_step_m": .005, "xy_tolerance_m": .02,
        "yaw_tolerance_rad": math.radians(1)}
    checks["current_source"] = all(
        digest(ROOT/p) == state["sha256"] for p, state in run["code_state"].items())
    checks["execution_certificate"] = (run["status"] == "pass" and
        report["status"] == "FEASIBLE" and selected["status"] == "FEASIBLE" and
        selected["geometry_status"] == "PASS" and
        selected["dynamic_status"] == "PASS" and selected["timing_status"] == "PASS")
    checks["published_trace"] = (probe["terminal_ok"] is True and
        probe["timed_sweep_clear"] is True and probe["strictly_increasing_time"] is True and
        probe["independent_dynamics"]["pass_check"] is True and
        probe["independent_direction"]["pass_check"] is True)
    checks["budget"] = (report["budget_exceeded"] is False and
                         report["total_s"] <= report["budget_s"])
    checks["no_extra_stops"] = (timing["same_mode_hold_seconds"] == 0 and
                                 timing["gear_change_hold_seconds"] == 0)
    checks["cleanup"] = all(run["cleanup"].get(k) is True for k in
        ("ros_master_stopped", "port_released", "all_started_processes_stopped"))
    display = audit_display(probe)
    checks["display_messages"] = display["pass_check"]
    ordinary = [x for x in timing["low_speed_intervals"]
                if x["classification"] == "ordinary_translation"]
    checks["ordinary_low_speed_explained"] = all(x["active_speed_limits"] for x in ordinary)
    distance = 0.
    crossing_distance = 0.
    for a, b in zip(probe["timed_trace"], probe["timed_trace"][1:]):
        va, vb = abs(a[4]), abs(b[4])
        ds = math.hypot(b[1]-a[1], b[2]-a[2])
        if .5*(va+vb) < .05:
            distance += ds
        fraction = (1. if max(va, vb) < .05 else
                    0. if min(va, vb) >= .05 else
                    (.05-min(va, vb))/abs(vb-va))
        crossing_distance += ds*fraction
    coarse = re.search(r"\bseconds=([-+0-9.eE]+)", probe.get("search", ""))
    probe_wall = (datetime.fromisoformat(run["finished_at_utc"])-
                  datetime.fromisoformat(run["started_at_utc"])).total_seconds()
    return {"case": case["case"], "run_dir": str(path.relative_to(ROOT)),
            "probe_sha256": digest(path/"probe_result.json"),
            "checks": checks, "all_checks_pass": all(checks.values()),
            "score": selected["score"], "duration_s": timing["duration_s"],
            "mode_switches": timing["mode_switch_count"],
            "backend_s": report["total_s"], "budget_s": report["budget_s"],
            "cpp_planning_s": float(coarse.group(1)) if coarse else None,
            "first_execution_feasible_s": report["first_feasible_s"],
            "geometry_certified_by_s": report["first_feasible_s"],
            "geometry_time_observation": "first complete independent execution audit includes geometry certification",
            "optimizer_callback_wall_s": report.get("optimizer_callback_wall_s"),
            "probe_total_wall_s_including_startup_and_cleanup": probe_wall,
            "source": report["selected_source"],
            "distance_below_0p05_mps_midpoint_sampled_m": distance,
            "distance_below_0p05_mps_linear_crossing_sampled_m": crossing_distance,
            "ordinary_low_speed_total_s": sum(x["duration_s"] for x in ordinary),
            "ordinary_low_speed_longest_s": max([x["duration_s"] for x in ordinary]+[0.]),
            "display": display}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    args = parser.parse_args()
    base = args.artifact_dir.resolve()
    cases = yaml.safe_load((base/"final_validation_cases.yaml").read_text())["cases"]
    repeats = yaml.safe_load((base/"final_repeat_cases.yaml").read_text())["cases"]
    rows = [inspect_run(base/"final_v2_regression"/name, case)
            for name, case in cases.items()]
    repeated = [inspect_run(base/"final_critical_repeats"/name, case)
                for name, case in repeats.items()]
    thresholds = yaml.safe_load((BASE/"cases.yaml").read_text())["decisions"][
        "threshold_policy"]["frozen_absolute_targets"]
    critical = {}
    for name in ("figure3", "recent_153s_crawl"):
        trials = [r for r in rows+repeated if
                  r["case"] == name or r["case"].startswith(name+"_r")]
        passed = []
        for r in trials:
            if name == "figure3":
                ok = r["duration_s"] <= 78 and r["mode_switches"] <= 11 and r["score"] <= 32.954
            else:
                ok = (r["duration_s"] <= thresholds["recent_153s_crawl_duration_s_max"] and
                      max(r["distance_below_0p05_mps_midpoint_sampled_m"],
                          r["distance_below_0p05_mps_linear_crossing_sampled_m"]) <=
                      thresholds["recent_153s_crawl_distance_below_0p05_m_max"] and
                      r["ordinary_low_speed_total_s"] <=
                      thresholds["recent_153s_crawl_ordinary_translation_low_speed_total_s_max"] and
                      r["ordinary_low_speed_longest_s"] <=
                      thresholds["recent_153s_crawl_ordinary_translation_low_speed_event_s_max"])
            passed.append(bool(ok and r["all_checks_pass"]))
        critical[name] = {"trial_count": len(trials), "pass_count": sum(passed),
                          "pass": len(trials) == 5 and all(passed), "trials": trials}
    legacy = read(base/"shared_final_geometry_replay_final.json")
    reference_profile = read(base/"final_v2_regression/figure1/probe_result.json")[
        "quality_report"]["selected"]["timing"]["profile"]
    legacy_pass = (legacy["case_count"] == 3 and legacy["strict_geometry_pass_count"] == 3 and
                   legacy["shared_execution_pass_count"] == 3 and legacy["downstream_nlp_disabled"])
    for c in legacy["cases"]:
        s = c["shared_execution"]
        legacy_pass &= (digest(ROOT/c["source_final_report"]) ==
                        c["source_final_report_sha256"] and
                        c["map_signature_verified"] and s["status"] == "FEASIBLE" and
                        s["audit"]["geometry_status"] == "PASS" and
                        s["audit"]["dynamic_status"] == "PASS" and
                        s["audit"]["timing"]["profile"] == reference_profile and
                        c["shared_replay_config"]["max_nlp_calls"] == 0 and
                        not s["report"]["budget_exceeded"])
    gates = {"fifteen_frozen_inputs": len(rows) == 15 and all(r["all_checks_pass"] for r in rows),
             "critical_repeats": all(v["pass"] for v in critical.values()),
             "nonempty_legacy_preservation": bool(legacy_pass)}
    negative_path = base/"negative_geometry_only_N02/run.json"
    if negative_path.exists():
        negative = read(negative_path)
        gates["geometry_only_failure_contract"] = (
            negative["status"] == "pass" and negative["expected_outcome"] == "GEOMETRY_ONLY" and
            negative["probe_result"]["geometry_only_check"]["pass_check"] is True and
            negative["probe_result"]["quality_report"]["budget_exceeded"] is False and
            all(negative["cleanup"].get(k) is True for k in
                ("ros_master_stopped", "port_released", "all_started_processes_stopped")) and
            all(digest(ROOT/p) == state["sha256"]
                for p, state in negative["code_state"].items()))
    else:
        gates["geometry_only_failure_contract"] = False
    verification_path = base/"command_verification.json"
    if verification_path.exists():
        verification = read(verification_path)
        gates["unit_tests_and_build"] = (all(c["exit_code_observed"] == 0 for c in
            verification["commands"]) and all(digest(ROOT/e["path"]) == e["sha256"]
            for e in verification["files_unchanged_since_execution"]))
    else:
        gates["unit_tests_and_build"] = False
    freeze = read(BASE/"historical_legacy_preservation_20261018/freeze_manifest.json")
    gates["legacy_inputs_unchanged_since_freeze"] = all(
        digest(ROOT/e["path"]) == e["sha256"] for e in freeze["files"])
    cpp_path = base/"planner_policy_history_gtest.xml"
    if cpp_path.exists():
        cpp = ET.parse(cpp_path).getroot()
        gates["cpp_policy_history"] = (len(list(cpp.iter("testcase"))) == 4 and
            not list(cpp.iter("failure")) and not list(cpp.iter("error")) and
            not list(cpp.iter("skipped")))
    else:
        gates["cpp_policy_history"] = False
    manifest_path = BASE/"after_anchor_mode_refactor/source_manifest.json"
    manifest = read(manifest_path)
    manifest_entries = manifest["files"]
    expected_tree = hashlib.sha256(json.dumps(manifest_entries, sort_keys=True,
        separators=(",", ":")).encode()).hexdigest()
    gates["source_release_matches_workspace"] = (
        expected_tree == manifest["tree_sha256"] and all(
            digest(ROOT/e["path"]) == e["sha256"] and
            (ROOT/e["path"]).stat().st_size == e["size_bytes"] for e in manifest_entries))
    recorded_runs = [read(p) for p in base.rglob("run.json")]
    gates["all_recorded_runs_cleaned"] = (len(recorded_runs) >= 25 and all(
        all(r["cleanup"].get(k) is True for k in
            ("ros_master_stopped", "port_released", "all_started_processes_stopped"))
        for r in recorded_runs))
    result = {"schema": "r12_final_automated_acceptance_v1",
              "created_at_utc": datetime.now(timezone.utc).isoformat(),
              "scope": "frozen fifteen inputs, restricted profile, current source; no all-map or hardware claim",
              "gates": gates, "all_automated_gates_pass": all(gates.values()),
              "frozen_runs": rows, "critical": critical,
              "legacy_shared_replay_path": str((base/"shared_final_geometry_replay_final.json").relative_to(ROOT)),
              "legacy_shared_replay_sha256": digest(base/"shared_final_geometry_replay_final.json"),
              "legacy_native_clock_restricted_certified": False,
              "geometry_only_negative_run": str(negative_path.relative_to(ROOT)),
              "source_manifest": str(manifest_path.relative_to(ROOT)),
              "source_manifest_sha256": digest(manifest_path),
              "source_tree_sha256": manifest["tree_sha256"],
              "source_file_count": len(manifest_entries),
              "recorded_ros_run_count": len(recorded_runs),
              "low_speed_distance_method": "sum pose chord when interval mean abs(v)<0.05; also report linear threshold-crossing estimate",
              "warnings": ["N01 duration depends on upstream warm seed and proposal ranking; faster historical diagnostic candidate is not a frozen time gate.",
                           "IPOPT request is not a hard wall-time guarantee; shared backend budget is audited."]}
    (base/"final_acceptance_summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"gates": gates, "all_automated_gates_pass": all(gates.values())}))
    return 0 if all(gates.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
