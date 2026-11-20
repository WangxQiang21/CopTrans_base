#!/usr/bin/env python3
"""Freeze the TASK03 evidence tree and write the final manifest.

The manifest records the git identity, the byte-exact status/diff of the
worktree, the protected TASK01 hashes, every evidence file's SHA-256 and
size, the live-run check summaries and the offline check exit codes.  It
never touches TASK01/TASK02 evidence and never writes outside ``final/``.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path("/home/wq/whut_thesis/expv3_multi_virtual_a")
EVIDENCE = ROOT / "work_flow/TASK03_evidence/20261119T133018Z"
FINAL = EVIDENCE / "final"
PROTECTED = {
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/geometry.py":
        "f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d",
    "tests/test_virtual_a_geometry.py":
        "4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7",
}


def run(command):
    completed = subprocess.run(
        command, cwd=str(ROOT), stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=300)
    return {
        "command": command,
        "exit": completed.returncode,
        "stdout": completed.stdout.decode("utf-8", "replace"),
    }


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def evidence_files():
    entries = []
    for path in sorted(EVIDENCE.rglob("*")):
        if not path.is_file():
            continue
        if "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        if path in (FINAL / "SHA256SUMS", FINAL / "manifest.json"):
            continue
        entries.append({
            "path": str(path.relative_to(EVIDENCE)),
            "size": path.stat().st_size,
            "sha256": sha256(path),
        })
    return entries


def live_runs():
    runs = []
    for summary_path in sorted((EVIDENCE / "step6/live").glob(
            "*/capture_summary.json")):
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        runs.append({
            "run": summary_path.parent.name,
            "mode": summary.get("mode"),
            "passed": summary.get("passed"),
            "errors": summary.get("errors"),
            "failed_checks": sorted(
                name for name, entry in
                (summary.get("checks") or {}).items()
                if not entry.get("ok")),
            "executable_envelope_count": summary.get(
                "executable_envelope_count"),
            "planner_scope": (summary.get("notes") or {}).get(
                "planner_scope"),
            "union_binding_summary": (summary.get("notes") or {}).get(
                "union_binding_summary"),
            "negative_evidence": (summary.get("notes") or {}).get(
                "negative_evidence"),
            "terminal_state": (
                (summary.get("job_history") or [{}])[-1].get("state")
                if summary.get("job_history") else None),
        })
    return runs


def check_exit_codes():
    directory = EVIDENCE / "step6/tests"
    entries = {}
    for path in sorted(directory.glob("*.exit")):
        entries[path.stem] = int(path.read_text().strip())
    return entries


def main():
    FINAL.mkdir(parents=True, exist_ok=True)
    status = run(["git", "status", "--porcelain=v1"])
    diff_stat = run(["git", "diff", "--stat"])
    head = run(["git", "rev-parse", "HEAD"]).get("stdout", "").strip()
    branch = run(["git", "rev-parse", "--abbrev-ref", "HEAD"]).get(
        "stdout", "").strip()
    remotes = run(["git", "remote", "-v"]).get("stdout", "").strip()

    protected = {}
    for relative, expected in PROTECTED.items():
        actual = sha256(ROOT / relative)
        protected[relative] = {
            "expected": expected, "actual": actual,
            "unchanged": actual == expected,
        }

    changed = []
    for line in status["stdout"].splitlines():
        if len(line) < 4:
            continue
        relative = line[3:].strip()
        if relative.startswith('"') and relative.endswith('"'):
            relative = relative[1:-1].encode().decode("unicode_escape")
        path = ROOT / relative
        if path.is_file() and not relative.startswith("work_flow/"):
            changed.append({
                "status": line[:2].strip(),
                "path": relative,
                "sha256": sha256(path),
                "size": path.stat().st_size,
            })

    manifest = {
        "schema": "task03.step6.final_manifest.v1",
        "generated_at_utc": run(
            ["date", "-u", "+%Y-%m-%dT%H:%M:%SZ"])["stdout"].strip(),
        "worktree": str(ROOT),
        "git": {
            "head": head,
            "branch": branch,
            "remotes": remotes,
            "status_porcelain": status["stdout"].splitlines(),
            "diff_stat": diff_stat["stdout"],
        },
        "protected_task01_files": protected,
        "changed_repository_files": changed,
        "live_runs": live_runs(),
        "offline_check_exit_codes": check_exit_codes(),
        "evidence_files": evidence_files(),
    }
    manifest_path = FINAL / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=1, sort_keys=True) + "\n",
        encoding="utf-8")

    (FINAL / "git_status_end_step6.txt").write_text(
        status["stdout"], encoding="utf-8")
    (FINAL / "git_diff_stat_end_step6.txt").write_text(
        diff_stat["stdout"], encoding="utf-8")
    lines = []
    for relative, values in protected.items():
        lines.append("{}  {}".format(values["actual"], relative))
    for entry in changed:
        lines.append("{}  {}".format(entry["sha256"], entry["path"]))
    (FINAL / "source_hashes_end_step6.txt").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")

    sums = []
    for entry in evidence_files():
        if entry["path"] == "final/SHA256SUMS":
            continue
        sums.append("{}  {}".format(entry["sha256"], entry["path"]))
    sums.append("{}  final/manifest.json".format(sha256(manifest_path)))
    (FINAL / "SHA256SUMS").write_text(
        "\n".join(sums) + "\n", encoding="utf-8")
    print("manifest written: {} ({} evidence files, {} live runs)".format(
        manifest_path, len(manifest["evidence_files"]),
        len(manifest["live_runs"])))
    return 0 if all(values["unchanged"]
                    for values in protected.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())