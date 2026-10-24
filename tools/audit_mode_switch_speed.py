#!/usr/bin/env python3
"""Check actual published speed traces against the interactive timing policy."""
import argparse
import json
from pathlib import Path

import numpy as np


def audit_run(path):
    run = json.loads(path.read_text())
    probe = run.get("probe_result", {})
    report = probe.get("quality_report", {})
    timing = report.get("selected_timing", {})
    profile = timing.get("profile", {})
    trace = np.asarray(probe.get("timed_trace", []), dtype=float)
    result = {"case": run.get("case"), "run": str(path),
              "duration_s": timing.get("duration_s"), "checks": {}}
    checks = result["checks"]
    checks["successful_probe"] = run.get("status") == "pass"
    checks["requested_policy"] = (
        profile.get("translation_speed_policy") == "mode_switch_only" and
        profile.get("mode_switch_duration") == .3)
    checks["swept_clear_and_terminal"] = (
        probe.get("timed_sweep_clear") is True and probe.get("terminal_ok") is True)
    checks["audited_timing"] = (
        report.get("selected", {}).get("timing_status") == "PASS" and
        report.get("selected", {}).get("dynamic_status") == "PASS")
    switches = [p for p in timing.get("phases", []) if p.get("kind") == "mode_switch"]
    result["switch_durations_s"] = [p["duration"] for p in switches]
    checks["all_switches_point_three"] = all(abs(p["duration"]-.3) < 1e-9 for p in switches)
    checks["no_extra_holds"] = (
        timing.get("same_mode_hold_seconds") == 0. and
        timing.get("gear_change_hold_seconds") == 0. and
        abs(timing.get("mode_switch_hold_seconds", -1.)-.3*len(switches)) < 1e-8)
    if trace.ndim != 2 or trace.shape[1] != 6:
        checks["published_trace"] = False
        result["all_pass"] = False
        return result
    checks["published_trace"] = len(trace) > 1 and bool(np.all(np.isfinite(trace)))
    checks["observed_switch_stops"] = True
    for switch in switches:
        t0, t1 = switch["t0"], switch["t0"]+switch["duration"]
        first = int(np.argmin(abs(trace[:, 0]-t0)))
        last = int(np.argmin(abs(trace[:, 0]-t1)))
        rows = trace[first:last+1]
        checks["observed_switch_stops"] &= bool(
            abs(trace[first, 0]-t0) < 2e-8 and abs(trace[last, 0]-t1) < 2e-8 and
            abs(trace[last, 0]-trace[first, 0]-.3) < 2e-8 and
            np.max(abs(rows[:, 4])) < 1e-7 and
            np.max(abs(rows[:, 1:4]-rows[0, 1:4])) < 1e-7)
    groups = {}
    for phase in timing.get("phases", []):
        if phase.get("kind") == "translation":
            groups.setdefault(phase["block_index"], []).append(phase)
    blocks = []
    for index, phases in groups.items():
        t0 = phases[0]["t0"]
        t1 = phases[-1]["t0"]+phases[-1]["duration"]
        rows = trace[(trace[:, 0] >= t0-2e-8) & (trace[:, 0] <= t1+2e-8)]
        speeds = abs(rows[:, 4])
        changes = np.diff(speeds)
        braking = np.flatnonzero(changes < -2e-5)
        unimodal = (not len(braking) or not np.any(changes[braking[0]:] > 2e-5))
        blocks.append({"block_index": index, "peak_speed_mps": float(max(speeds)),
                       "no_interior_brake_then_accelerate": bool(unimodal),
                       "no_interior_stop": bool(np.all(speeds[1:-1] > 1e-7))})
    result["translation_blocks"] = blocks
    checks["no_curve_speed_dips"] = bool(blocks) and all(
        b["no_interior_brake_then_accelerate"] and b["no_interior_stop"] for b in blocks)
    constraints = timing.get("speed_constraint_intervals", [])
    checks["no_geometry_speed_caps"] = bool(constraints) and all(
        (set(row.get("speed_caps_mps", {})) == {"target_speed"} or
         row.get("cap_source") == "rest_to_rest_dense_dynamic_peaks") for row in constraints)
    checks["target_not_exceeded"] = bool(np.max(abs(trace[:, 4])) <= profile.get("target_v", 0.)+1e-6)
    checks["independent_direction"] = probe.get("independent_direction", {}).get("pass_check") is True
    checks["cleaned_up"] = all(run.get("cleanup", {}).get(key) is True for key in
        ("port_released", "all_started_processes_stopped", "ros_master_stopped"))
    result["all_pass"] = all(checks.values())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    results = [audit_run(p) for p in args.runs]
    output = {"runs": results, "all_pass": all(r["all_pass"] for r in results)}
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2)+"\n")
    for result in results:
        print(json.dumps({key: result[key] for key in
                          ("case", "duration_s", "switch_durations_s", "all_pass")}))
        if not result["all_pass"]:
            print({k: v for k, v in result["checks"].items() if not v})
    return 0 if output["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
