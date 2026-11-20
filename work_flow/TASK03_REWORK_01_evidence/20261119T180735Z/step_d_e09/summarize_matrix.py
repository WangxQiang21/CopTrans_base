#!/usr/bin/env python3
"""Summarize the E09 scene-feasibility matrix from the saved profiling logs.

Reads only the logs in this directory; writes matrix_summary.json and
matrix_summary.md next to them.  No solver is run.
"""
import json
import math
import pathlib
import re


HERE = pathlib.Path(__file__).resolve().parent


def _guide_length(points):
    return math.fsum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:]))


def _parse(path):
    text = path.read_text(encoding="utf-8")
    row = {"log": path.name}
    match = re.search(r"^GUIDE (\[.*\])$", text, re.M)
    row["guide"] = eval(match.group(1)) if match else None
    if row["guide"]:
        row["guide_length_m"] = round(_guide_length(row["guide"]), 3)
    match = re.search(r"^ALLOC (\[.*\])$", text, re.M)
    allocation = eval(match.group(1)) if match else []
    row["allocation"] = allocation
    switches = []
    for left, right in zip(allocation, allocation[1:]):
        if left[2] != right[2]:
            switches.append([left[2], right[2], left[1]])
    row["allocation_switches"] = switches
    match = re.search(
        r"^== CAPTURE 0 (\w+) cert (\w+) .*? L ([\d.]+)", text, re.M)
    if match:
        row["capture_source"] = match.group(1)
        row["certificate_status"] = match.group(2)
        row["path_length_m"] = float(match.group(3))
        if row.get("guide_length_m"):
            row["length_ratio_path_over_guide"] = round(
                row["path_length_m"] / row["guide_length_m"], 4)
    segs = re.search(r"^SEGS (\[.*\])$", text, re.M)
    row["template_segments"] = eval(segs.group(1)) if segs else None
    worst = re.search(
        r"^WORST r_alloc \(nominal alloc region\) (-?[\d.eE+]+) at sigma "
        r"([\d.]+) F \(([-\d.]+), ([-\d.]+)\)", text, re.M)
    if worst:
        row["worst_allocation_point_residual_m"] = float(worst.group(1))
        row["worst_allocation_sigma"] = float(worst.group(2))
        row["worst_allocation_f"] = (
            float(worst.group(3)), float(worst.group(4)))
    old = re.search(
        r"^OLD_AUDIT passed (\w+) (\w+) max_bound (-?[\d.eE+]+|-?inf|None)",
        text, re.M)
    if old:
        row["old_audit_passed"] = old.group(1) == "True"
        row["old_audit_reason"] = old.group(2)
        try:
            row["old_audit_max_bound_m"] = float(old.group(3))
        except ValueError:
            row["old_audit_max_bound_m"] = None
    ff = re.search(r"^OLD_AUDIT .* ff \{.*?\}$", text, re.M)
    row["old_audit_first_failure"] = (
        "None" if ff is None else re.search(r" ff (\{.*\})$", ff.group(0)).group(1))
    unclosed = re.search(r"^FIRST UNCLOSED (\S+) (\d+) (\[.*\])$", text, re.M)
    if unclosed:
        row["first_unclosed_s_m"] = float(unclosed.group(1))
        row["first_unclosed_open"] = eval(unclosed.group(3))
    result = re.search(
        r"^RESULT (\S+) (\S+) exec (\w+)$", text, re.M)
    if result:
        row["pipeline_status"] = result.group(1)
        row["pipeline_reason"] = result.group(2)
        row["pipeline_executable"] = result.group(3) == "True"
    attempts = re.search(r"^ATTEMPTS (\[.*\])$", text, re.M)
    row["template_attempts"] = eval(attempts.group(1)) if attempts else None
    exits = re.findall(r"^(?:EXIT|Traceback|.*Error:.*)$", text, re.M)
    row["exit_marker"] = exits[-1] if exits else None
    return row


def main():
    rows = [_parse(path) for path in sorted(HERE.glob("m*.log"))]
    blockers = [_parse(path) for path in sorted(HERE.glob("attempt_block_*.log"))]
    summary = {
        "schema": "task03_rework01.e09_matrix_summary.v1",
        "scene_count": len(rows),
        "scenes": rows,
        "obstacle_attempts": blockers,
    }
    (HERE / "matrix_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    lines = [
        "# E09 scene-feasibility matrix summary",
        "",
        "| log | path L | guide L | L/path-guide | alloc switch | "
        "template F segs | worst r_alloc (m) | old audit | old max bound | cert |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for row in rows + blockers:
        switch = ", ".join(
            "{}@{}".format(right, cut) for _left, right, cut
            in row.get("allocation_switches") or [])
        segs = row.get("template_segments")
        segs_text = "" if not segs else " ".join(
            "[{:.3f},{:.3f}]f{}".format(s, e, f) for s, e, f in segs)
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            row["log"],
            row.get("path_length_m", ""),
            row.get("guide_length_m", ""),
            row.get("length_ratio_path_over_guide", ""),
            switch,
            segs_text,
            row.get("worst_allocation_point_residual_m", ""),
            "PASS" if row.get("old_audit_passed") else
            ("FAIL" if "old_audit_passed" in row else "n/a"),
            row.get("old_audit_max_bound_m", ""),
            row.get("certificate_status", row.get("pipeline_reason", "")),
        ))
    (HERE / "matrix_summary.md").write_text("\n".join(lines) + "\n",
                                             encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())