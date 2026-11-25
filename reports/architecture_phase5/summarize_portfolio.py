#!/usr/bin/env python3
"""Aggregate matched ROS planning-only results without treating repeats as cases."""

import argparse
import json
import math
from pathlib import Path
from statistics import median


def grouped(path):
    cases = {}
    for row in json.loads(path.read_text())["scenarios"]:
        if not row.get("warmup"):
            cases.setdefault(row["case"], []).append(row)
    return cases


def metrics(rows):
    accepted = [row for row in rows if row["trajectory_count"] == 1]
    report = {"trials": len(rows), "certified": len(accepted),
              "timeouts": sum(row["timeout"] for row in rows),
              "duplicate_routes": sum(row["trajectory_count"] > 1 for row in rows)}
    if accepted:
        delays = sorted(row["request_to_wire_s"] for row in accepted)
        report.update(p50_plan_s=median(delays), max_plan_s=delays[-1],
                      reference_median_s=median(row["reference_duration_s"] for row in accepted),
                      max_position_error_m=max(row["terminal_position_error_m"] for row in accepted),
                      max_yaw_error_rad=max(row["terminal_yaw_error_rad"] for row in accepted))
        if len(delays) >= 30:
            report["p95_plan_s"] = delays[math.ceil(.95 * len(delays)) - 1]
        if len(delays) >= 100:
            report["p99_plan_s"] = delays[math.ceil(.99 * len(delays)) - 1]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("legacy", type=Path)
    parser.add_argument("portfolio", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    old, new = grouped(args.legacy), grouped(args.portfolio)
    if set(old) != set(new) or any(len(old[key]) != len(new[key]) for key in old):
        raise ValueError("ROS matrix does not contain all matched cases")
    report = {"cases": {}, "case_count": len(old)}
    for key in old:
        report["cases"][key] = {"legacy": metrics(old[key]), "portfolio": metrics(new[key])}
    for tag in ("legacy", "portfolio"):
        report[f"{tag}_certified_cases"] = sum(row[tag]["certified"] > 0
                                               for row in report["cases"].values())
    common = [row for row in report["cases"].values() if row["legacy"]["certified"]
              and row["portfolio"]["certified"]]
    before = median(row["legacy"]["p50_plan_s"] for row in common)
    after = median(row["portfolio"]["p50_plan_s"] for row in common)
    report.update(common_certified_cases=len(common), legacy_case_median_s=before,
                  portfolio_case_median_s=after, median_rosside_relative_reduction=1. - after / before)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
