#!/usr/bin/env python3
"""Serial isolated ROS regression for the refactored mode/path backend."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", default=["N06_default", "P12_env2_x0p8"])
    parser.add_argument("--port-base", type=int, default=11580)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cases-file", type=Path,
                        help="optional frozen task manifest passed to each probe")
    parser.add_argument("--backend", choices=("mode_path_v1", "mode_path_v2",
                                               "legacy", "case"),
                        default="mode_path_v1",
                        help="backend override; 'case' keeps the launch config default")
    args = parser.parse_args()
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, case in enumerate(args.cases):
        run_dir = destination / case
        command = [sys.executable, str(ROOT/"tools/run_expv3_probe.py"), "--case", case,
                   "--port", str(args.port_base+index), "--joint", "off", "--cbc", "off"]
        if args.cases_file:
            command.extend(["--cases-file", str(args.cases_file.resolve())])
        if args.backend != "case":
            command.extend(["--set", f"planning.backend={args.backend}"])
        command.extend(["--output-dir", str(run_dir)])
        process = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        data = json.loads((run_dir/"run.json").read_text()) if (run_dir/"run.json").exists() else {}
        probe = data.get("probe_result", {})
        report = probe.get("quality_report", {})
        candidates = report.get("candidates", [])
        initial = next((c["audit"] for c in candidates if c["source"] == "coarse_incumbent"), {})
        selected = report.get("selected", {})
        row = {"case": case, "run_dir": str(run_dir), "exit_code": process.returncode,
               "status": data.get("status", "runner_failed"),
               "baseline": "same_cpp_seed_retimed_under_quality_profile",
               "initial_score": initial.get("score"), "selected_score": selected.get("score"),
               "initial_metrics": initial.get("metrics"), "selected_metrics": selected.get("metrics"),
               "initial_duration_s": initial.get("timing", {}).get("duration_s"),
               "selected_duration_s": selected.get("timing", {}).get("duration_s"),
               "mode_runs": probe.get("mode_runs"), "independent_dynamics": probe.get("independent_dynamics"),
               "swept_clear": probe.get("timed_sweep_clear"), "sweep_samples": probe.get("sweep_samples"),
               "terminal_ok": probe.get("terminal_ok"),
               "reverse_body_projection": probe.get("reverse_body_projection_detected"),
               "independent_direction": probe.get("independent_direction"),
               "backend_seconds": report.get("total_s"), "first_feasible_s": report.get("first_feasible_s"),
               "budget_exceeded": report.get("budget_exceeded"),
               "port_released": data.get("cleanup", {}).get("port_released"),
               "processes_stopped": data.get("cleanup", {}).get("all_started_processes_stopped")}
        row["selected_source"] = report.get("selected_source")
        row["adaptive_budget_trigger"] = report.get("adaptive_budget_trigger")
        row["adaptive_budget_extension_s"] = report.get("adaptive_budget_extension_s")
        row["mode_anchor_base_switches"] = report.get("mode_anchor_base_switches")
        row["selected_mode_switches"] = report.get("selected_timing", {}).get("mode_switch_count")
        row["selected_duration_s"] = report.get("selected_timing", {}).get("duration_s")
        row["cleanup"] = data.get("cleanup", {})
        row["execution_profile"] = report.get("selected_timing", {}).get("profile", {})
        if initial.get("score") is not None and selected.get("score") is not None:
            row["score_improvement_fraction"] = (initial["score"]-selected["score"])/max(abs(initial["score"]), 1e-12)
        rows.append(row)
        cost_profile = {"mode_path_v1": "quality_cost_v1_initial",
                        "mode_path_v2": "quality_cost_v2_prototype_uncalibrated",
                        "legacy": "legacy_backend_default",
                        "case": "launch_config_default"}[args.backend]
        summary = {"schema": "mode_path_regression_v1", "cost_profile": cost_profile,
                   "backend_mode": args.backend,
                   "case_manifest": str(args.cases_file.resolve()) if args.cases_file else None,
                   "execution_profile": "per_run_selected_timing_profile",
                   "hardware_claim": False, "rows": rows, "all_pass": all(
                       r["status"] == "pass" and r["swept_clear"] and r["terminal_ok"] and
                       r.get("independent_direction", {}).get("pass_check") and
                       r.get("independent_dynamics", {}).get("pass_check") and
                       r["budget_exceeded"] is not True and
                       r["port_released"] and r["processes_stopped"] for r in rows)}
        (destination/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
        print(json.dumps({k: row.get(k) for k in ("case", "status", "initial_score", "selected_score",
                                                 "score_improvement_fraction", "backend_seconds")}), flush=True)
        if process.returncode:
            print(process.stdout[-3000:], flush=True)
    return 0 if summary["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
