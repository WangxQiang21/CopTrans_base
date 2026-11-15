#!/usr/bin/env python3
"""Regenerate and verify the FR02-A R3 rework evidence bundle.

This script does not modify source, tests or original evidence.  It runs the
seven regression suites and both copied Codex probes, writes their logs and
exit codes under ``rework_02``, refreshes the final source copies, diffs,
hashes and machine-readable self checks, and finally writes
``evidence_manifest.json``.

Run from any directory with::

    python3 evaluation/formation_refactor_v02/FR02/A/rework_02/checks/generate_evidence.py

Pass ``--manifest-only`` to refresh only ``evidence_manifest.json`` after a
report edit; this is used once the final report has been updated.
"""
import argparse
import hashlib
import json
import math
import os
import platform
import re
import shlex
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def _find_root():
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "catkin_ws").is_dir() and (parent / "tests").is_dir():
            return parent
    raise RuntimeError("worktree root not found from {}".format(here))


ROOT = _find_root()
R = Path(__file__).resolve().parent.parent
A = ROOT / "evaluation" / "formation_refactor_v02" / "FR02" / "A"
P = ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src" / "fwis_v3_optimizer"
PRE = R / "pre_rework"
PRE_SHARED = PRE / "shared"
FINAL = R / "final"
DIFF = R / "diff"
LOGS = R / "logs"
RESULTS = R / "results"
ARTIFACTS = R / "artifacts"
CHECKS = R / "checks"
PROBE = R / "codex_probe"
DECODER_PROBE = R / "codex_decoder_probe"
ORIGINAL_PROBE_DIR = A / "codex_acceptance_20261113T115526Z"
REWORK_01 = A / "rework_01"

PROBE_SCRIPT_NAME = "independent_probes.py"
DECODER_SCRIPT_NAME = "decoder_time_probes.py"
IND_ORIGINAL = PROBE / "original"
IND_PRE_FIX = PROBE / "pre_fix"
IND_POST_FIX = PROBE / "post_fix"
DEC_ORIGINAL = DECODER_PROBE / "original"
DEC_PRE_FIX = DECODER_PROBE / "pre_fix"
DEC_POST_FIX = DECODER_PROBE / "post_fix"

SHARED = (
    (P / "formation_geometry.py", "formation_geometry.py"),
    (P / "formation_kinematics.py", "formation_kinematics.py"),
    (P / "formation_retiming.py", "formation_retiming.py"),
    (P / "uniform_timing.py", "uniform_timing.py"),
    (P / "uniform_trajectory.py", "uniform_trajectory.py"),
    (ROOT / "tests" / "test_uniform_timing.py", "test_uniform_timing.py"),
    (ROOT / "tests" / "test_uniform_trajectory.py", "test_uniform_trajectory.py"),
)
EXPECTED_REWORK_CHANGED = {"uniform_trajectory.py",
                           "test_uniform_trajectory.py"}
EXPECTED_TRACKED_SHARED = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_geometry.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_kinematics.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_retiming.py",
)

SUITES = (
    {
        "name": "test_uniform_timing",
        "argv": ("tests/test_uniform_timing.py", "-v"),
        "env": {"FR02_TIMING_RESULTS_PATH":
                str(RESULTS / "timing_results.json")},
        "expected_tests": 5,
    },
    {
        "name": "test_uniform_trajectory",
        "argv": ("tests/test_uniform_trajectory.py", "-v"),
        "env": {
            "FR02_TRAJECTORY_RESULTS_PATH":
                str(RESULTS / "trajectory_results.json"),
            "FR02_ARTIFACT_DIR": str(ARTIFACTS),
        },
        "expected_tests": 8,
    },
    {
        "name": "test_uniform_motion",
        "argv": ("tests/test_uniform_motion.py", "-v"),
        "env": {"FR01_RESULTS_PATH":
                str(RESULTS / "fr01_primitive_results.json")},
        "expected_tests": 13,
    },
    {
        "name": "test_formation_geometry",
        "argv": ("tests/test_formation_geometry.py", "-v"),
        "env": {},
        "expected_tests": 15,
    },
    {
        "name": "test_formation_kinematics",
        "argv": ("tests/test_formation_kinematics.py", "-v"),
        "env": {},
        "expected_tests": 28,
    },
    {
        "name": "test_formation_retiming",
        "argv": ("tests/test_formation_retiming.py", "-v"),
        "env": {},
        "expected_tests": 14,
    },
    {
        "name": "test_formation_hold",
        "argv": ("tests/test_formation_hold.py", "-v"),
        "env": {},
        "expected_tests": 14,
    },
)

DIFF_FILES = (
    ("uniform_trajectory.py",
     "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_trajectory.py"),
    ("test_uniform_trajectory.py", "tests/test_uniform_trajectory.py"),
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def sha256(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_hash_file(path):
    entries = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        digest, name = line.split(None, 1)
        entries.append((digest, name.strip()))
    return entries


def run_git(*args):
    result = subprocess.run(["git", "-C", str(ROOT)] + list(args),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True)
    return result.returncode, result.stdout


def shell_command(argv, env_overrides):
    parts = ["env"]
    for key in sorted(env_overrides):
        parts.append("{}={}".format(key, shlex.quote(str(env_overrides[key]))))
    parts.extend(shlex.quote(str(item)) for item in argv)
    return " ".join(parts)


def run_command(name, argv, env_overrides, log_path, exit_path):
    env = os.environ.copy()
    env.update({key: str(value) for key, value in env_overrides.items()})
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    started = datetime.now(timezone.utc)
    result = subprocess.run(
        [sys.executable] + list(argv), cwd=str(ROOT), env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        timeout=600)
    finished = datetime.now(timezone.utc)
    log_path.write_text(result.stdout, encoding="utf-8")
    exit_path.write_text("{}\n".format(result.returncode), encoding="utf-8")
    return {
        "name": name,
        "argv": list(argv),
        "cwd": str(ROOT),
        "env_overrides": dict(env_overrides, OPENBLAS_NUM_THREADS="1",
                               PYTHONDONTWRITEBYTECODE="1"),
        "shell_command": shell_command([sys.executable] + list(argv),
                                       dict(env_overrides,
                                            OPENBLAS_NUM_THREADS="1",
                                            PYTHONDONTWRITEBYTECODE="1")),
        "log_path": str(log_path.relative_to(R)),
        "exit_code_path": str(exit_path.relative_to(R)),
        "exit_code": result.returncode,
        "expected_exit_code": 0,
        "started_at_utc": started.isoformat(),
        "finished_at_utc": finished.isoformat(),
    }


def copy_final_files():
    FINAL.mkdir(parents=True, exist_ok=True)
    (FINAL / "shared").mkdir(parents=True, exist_ok=True)
    for source, name in SHARED:
        shutil.copyfile(str(source), str(FINAL / "shared" / name))


def write_hashes():
    final_lines = []
    for source, name in SHARED:
        final_lines.append("{}  {}".format(sha256(source), name))
    (FINAL / "final_hashes.sha256").write_text(
        "\n".join(final_lines) + "\n", encoding="utf-8")
    (FINAL / "relative_hashes.sha256").write_text(
        "\n".join(final_lines) + "\n", encoding="utf-8")

    pre_absolute = {path: digest for digest, path in read_hash_file(
        PRE / "absolute_hashes.sha256")}
    prior_shared = {name: digest for digest, name in read_hash_file(
        REWORK_01 / "final" / "final_hashes.sha256")}
    prior_fr01 = {path: digest for digest, path in read_hash_file(
        REWORK_01 / "final" / "fr01_hashes.sha256")}
    previous_originals = {path: digest for digest, path in read_hash_file(
        PRE / "original_hashes.sha256")}

    def comparison(path, before, category):
        path = Path(path)
        if not path.exists():
            raise RuntimeError("baseline path missing: {}".format(path))
        after = sha256(path)
        return {
            "path": str(path),
            "relative_to_worktree": str(path.relative_to(ROOT)),
            "before_sha256": before,
            "after_sha256": after,
            "changed": before != after,
            "category": category,
        }

    comparisons = []
    for source, name in SHARED:
        before = pre_absolute.get(str(source))
        if before is None:
            before = prior_shared.get(name)
        if before is None:
            raise RuntimeError("no pre-R3 baseline for {}".format(source))
        comparisons.append(comparison(source, before, "shared"))
    fr01_paths = (P / "uniform_motion.py",
                  ROOT / "tests" / "test_uniform_motion.py")
    for path in fr01_paths:
        before = prior_fr01.get(str(path))
        if before is None:
            raise RuntimeError("no FR01 baseline for {}".format(path))
        comparisons.append(comparison(path, before, "fr01"))
    for path in (A / "REPORT.md", A / "CODEX_ACCEPTANCE.md"):
        before = previous_originals.get(str(path))
        if before is None:
            raise RuntimeError("no original baseline for {}".format(path))
        comparisons.append(comparison(path, before, "docs"))

    after_lines = ["{}  {}".format(item["after_sha256"], item["path"])
                   for item in comparisons]
    (FINAL / "absolute_hashes.sha256").write_text(
        "\n".join(after_lines) + "\n", encoding="utf-8")
    write_json(FINAL / "hashes_before_after.json", {
        "schema": "fr02a.rework_02.hashes_before_after.v1",
        "generated_at_utc": utc_now(),
        "worktree": str(ROOT),
        "files": comparisons,
    })

    fr01_lines = ["{}  {}".format(sha256(path), path) for path in fr01_paths]
    (FINAL / "fr01_hashes.sha256").write_text(
        "\n".join(fr01_lines) + "\n", encoding="utf-8")

    original_records = []
    for path, before in previous_originals.items():
        original_records.append(comparison(path, before, "original"))
    write_json(R / "original_evidence_comparison.json", {
        "schema": "fr02a.rework_02.original_evidence_comparison.v1",
        "generated_at_utc": utc_now(),
        "baseline": str((PRE / "original_hashes.sha256").relative_to(R)),
        "files": original_records,
        "unchanged": all(not item["changed"] for item in original_records),
    })

    by_name = {Path(item["path"]).name: item for item in comparisons}
    shared_changed = {
        name: bool(by_name[name]["changed"])
        for _, name in SHARED if name in by_name
    }
    return {
        "comparisons": comparisons,
        "by_name": by_name,
        "shared_changed": shared_changed,
        "original_records": original_records,
    }


def write_diffs():
    DIFF.mkdir(parents=True, exist_ok=True)
    records = []
    combined = []
    for name, relative in DIFF_FILES:
        pre_path = PRE_SHARED / name
        current_path = ROOT / relative
        output_path = DIFF / "{}.diff".format(name)
        command = [
            "diff", "-u", "--label", "a/" + relative,
            "--label", "b/" + relative, str(pre_path), str(current_path),
        ]
        result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        output_path.write_text(result.stdout, encoding="utf-8")
        exit_path = DIFF / "{}.diff.exit_code".format(name)
        exit_path.write_text("{}\n".format(result.returncode),
                             encoding="utf-8")
        command_path = DIFF / "{}.diff.command.txt".format(name)
        command_path.write_text(shlex.join(command) + "\n", encoding="utf-8")
        records.append({
            "file": name,
            "relative_path": relative,
            "command": shlex.join(command),
            "diff_path": str(output_path.relative_to(R)),
            "exit_code_path": str(exit_path.relative_to(R)),
            "exit_code": result.returncode,
            "expected_exit_code": 1,
            "diff_bytes": len(result.stdout.encode("utf-8")),
        })
        combined.append("diff --git a/{0} b/{0}\n{1}".format(
            relative, result.stdout))
    (DIFF / "complete.diff").write_text(
        "\n".join(combined), encoding="utf-8")
    return records


def parse_log(path):
    text = Path(path).read_text(encoding="utf-8")
    match = re.search(r"^Ran (\d+) tests? in ", text, re.MULTILINE)
    ran = int(match.group(1)) if match else None
    return {
        "ran_tests": ran,
        "ok": bool(re.search(r"^OK$", text, re.MULTILINE)),
        "tail": "\n".join(text.splitlines()[-6:]),
    }


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def collect_regression(commands):
    by_name = {item["name"]: item for item in commands}
    suites = []
    total = 0
    all_pass = True
    for expected in SUITES:
        name = expected["name"]
        command = by_name[name]
        parsed = parse_log(R / command["log_path"])
        passed = (
            command["exit_code"] == 0
            and parsed["ran_tests"] == expected["expected_tests"]
            and parsed["ok"]
        )
        total += parsed["ran_tests"] or 0
        all_pass = all_pass and passed
        suites.append({
            "name": name,
            "command": command["shell_command"],
            "log_path": command["log_path"],
            "exit_code": command["exit_code"],
            "expected_exit_code": 0,
            "ran_tests": parsed["ran_tests"],
            "expected_tests": expected["expected_tests"],
            "unittest_status": "OK" if parsed["ok"] else "FAILED",
            "log_tail": parsed["tail"],
            "passed": passed,
        })

    timing = read_json(RESULTS / "timing_results.json")
    trajectory = read_json(RESULTS / "trajectory_results.json")
    fr01 = read_json(RESULTS / "fr01_primitive_results.json")

    def result_summary(payload):
        cases = payload.get("cases", [])
        return {
            "status": payload.get("status"),
            "case_count": len(cases),
            "cases": [{
                "case": case.get("case"),
                "status": case.get("status"),
            } for case in cases],
            "all_cases_passed": bool(cases) and all(
                case.get("status") == "PASSED" for case in cases),
        }

    payload = {
        "schema": "fr02a.rework_02.regression_summary.v1",
        "generated_at_utc": utc_now(),
        "overall_passed": all_pass,
        "suites": suites,
        "total_tests": total,
        "expected_total_tests": sum(item["expected_tests"]
                                    for item in SUITES),
        "results": {
            "timing": result_summary(timing),
            "trajectory": result_summary(trajectory),
            "fr01": result_summary(fr01),
        },
    }
    write_json(R / "regression_summary.json", payload)
    return payload


def probe_stats(payload):
    boundaries = payload.get("boundary_evaluations", [])
    wheels = payload.get("S_wheel_velocity", [])
    interior = payload.get("interior_roundtrip", [])
    failed_boundaries = [item for item in boundaries
                         if item.get("status") != "PASSED"]
    failed_wheels = [item for item in wheels
                     if item.get("status") != "PASSED"]
    return {
        "explicit_boundary_failure_count":
            payload.get("boundary_failure_count"),
        "explicit_S_wheel_failure_count":
            payload.get("S_wheel_failure_count"),
        "interior_roundtrip_positive_count":
            payload.get("interior_roundtrip_positive_count"),
        "boundary_row_count": len(boundaries),
        "boundary_failed_row_count": len(failed_boundaries),
        "s_wheel_row_count": len(wheels),
        "s_wheel_failed_row_count": len(failed_wheels),
        "interior_row_count": len(interior),
        "interior_failed_row_count": sum(
            item.get("status") != "PASSED" for item in interior),
        "max_s_wheel_vector_error_mps": max(
            [item.get("wheel_velocity_error_mps", 0.0) for item in wheels]
            or [0.0]),
        "failed_boundary_labels": [
            {key: item.get(key) for key in
             ("plan", "phase", "label", "t", "reason", "detail")}
            for item in failed_boundaries
        ],
        "failed_s_wheel_rows": [
            {key: item.get(key) for key in
             ("vehicle_id", "wheel_index", "actual_signed_rolling",
              "expected_signed_rolling", "wheel_velocity_error_mps")}
            for item in failed_wheels
        ],
    }


def write_probe_comparison(probe_command):
    original = read_json(IND_ORIGINAL / "independent_probes.json")
    rework = read_json(IND_POST_FIX / "independent_probes.json")
    pre_fix = read_json(IND_PRE_FIX / "independent_probes.json")
    original_stats = probe_stats(original)
    rework_stats = probe_stats(rework)
    pre_fix_stats = probe_stats(pre_fix)

    def interior_requests(payload):
        return sorted(
            (item.get("plan"), item.get("phase"), repr(item.get("t")))
            for item in payload.get("interior_roundtrip", []))

    payload = {
        "schema": "fr02a.rework_02.probe_comparison.v1",
        "generated_at_utc": utc_now(),
        "probe_script": str((PROBE / PROBE_SCRIPT_NAME).relative_to(R)),
        "probe_script_sha256": sha256(PROBE / PROBE_SCRIPT_NAME),
        "original_probe_sha256":
            sha256(ORIGINAL_PROBE_DIR / PROBE_SCRIPT_NAME),
        "original_copy_json_sha256":
            sha256(IND_ORIGINAL / "independent_probes.json"),
        "pre_fix_json_sha256":
            sha256(IND_PRE_FIX / "independent_probes.json"),
        "post_fix_json_sha256":
            sha256(IND_POST_FIX / "independent_probes.json"),
        "original_copy_log_sha256":
            sha256(IND_ORIGINAL / "independent_probes.log"),
        "post_fix_log_sha256":
            sha256(IND_POST_FIX / "independent_probes.log"),
        "original": original_stats,
        "pre_fix": pre_fix_stats,
        "rework": rework_stats,
        "interior_request_set_identical":
            interior_requests(original) == interior_requests(rework),
        "interior_all_passed": (
            rework_stats["interior_row_count"] == 66
            and rework_stats["interior_failed_row_count"] == 0
            and rework_stats["interior_roundtrip_positive_count"] == 66),
        "boundary_green_preserved": (
            original_stats["boundary_row_count"] == 71
            and original_stats["boundary_failed_row_count"] == 0
            and rework_stats["boundary_row_count"] == 71
            and rework_stats["boundary_failed_row_count"] == 0),
        "s_wheel_green_preserved": (
            original_stats["s_wheel_row_count"] == 12
            and original_stats["s_wheel_failed_row_count"] == 0
            and rework_stats["s_wheel_row_count"] == 12
            and rework_stats["s_wheel_failed_row_count"] == 0),
        "pre_fix_still_green": (
            pre_fix_stats["boundary_failed_row_count"] == 0
            and pre_fix_stats["s_wheel_failed_row_count"] == 0),
        "rework_exit_code": probe_command["exit_code"],
        "rework_expected_exit_code": 0,
    }
    write_json(R / "probe_comparison.json", payload)
    return payload


def decoder_probe_stats(payload):
    return {
        "case_count": len(payload),
        "failed_case_count": sum(item.get("status") == "FAILED"
                                 for item in payload),
        "passed_case_count": sum(item.get("status") == "PASSED"
                                 for item in payload),
        "cases": [{
            "case": item.get("case"),
            "decoder": item.get("decoder"),
            "reason": item.get("reason"),
            "status": item.get("status"),
            "motion_t0": item.get("motion_t0"),
            "motion_duration": item.get("motion_duration"),
            "motion_end": item.get("motion_end"),
            "terminal_progress": item.get("terminal_progress"),
            "expected_progress": item.get("expected_progress"),
        } for item in payload],
    }


def write_decoder_probe_comparison(decoder_command):
    original = read_json(DEC_ORIGINAL / "decoder_time_probes.json")
    pre_fix = read_json(DEC_PRE_FIX / "decoder_time_probes.json")
    post_fix = read_json(DEC_POST_FIX / "decoder_time_probes.json")
    original_stats = decoder_probe_stats(original)
    pre_fix_stats = decoder_probe_stats(pre_fix)
    post_fix_stats = decoder_probe_stats(post_fix)
    expected_reasons = {
        "subtolerance_gap": "PHASE_TIME_DISCONTINUITY",
        "unrepresentable_motion_interval": "PHASE_TIME_UNREPRESENTABLE",
    }
    payload = {
        "schema": "fr02a.rework_02.decoder_probe_comparison.v1",
        "generated_at_utc": utc_now(),
        "probe_script": str(
            (DECODER_PROBE / DECODER_SCRIPT_NAME).relative_to(R)),
        "probe_script_sha256":
            sha256(DECODER_PROBE / DECODER_SCRIPT_NAME),
        "original_probe_sha256":
            sha256(ORIGINAL_PROBE_DIR / DECODER_SCRIPT_NAME),
        "original_copy_json_sha256":
            sha256(DEC_ORIGINAL / "decoder_time_probes.json"),
        "pre_fix_json_sha256":
            sha256(DEC_PRE_FIX / "decoder_time_probes.json"),
        "post_fix_json_sha256":
            sha256(DEC_POST_FIX / "decoder_time_probes.json"),
        "original_copy_log_sha256":
            sha256(DEC_ORIGINAL / "decoder_time_probes.log"),
        "pre_fix_log_sha256":
            sha256(DEC_PRE_FIX / "run.log"),
        "post_fix_log_sha256":
            sha256(DEC_POST_FIX / "decoder_time_probes.log"),
        "original": original_stats,
        "pre_fix": pre_fix_stats,
        "post_fix": post_fix_stats,
        "pre_fix_reproduces_two_failures": (
            pre_fix_stats["failed_case_count"] == 2
            and pre_fix_stats["passed_case_count"] == 0),
        "post_fix_both_rejected_at_decode": all(
            item.get("status") == "PASSED"
            and item.get("decoder") == "REJECTED"
            and item.get("reason") == expected_reasons.get(item.get("case"))
            for item in post_fix),
        "expected_decode_reasons": expected_reasons,
        "pre_fix_exit_code":
            int((DEC_PRE_FIX / "exit_code").read_text(
                encoding="utf-8").strip()),
        "post_fix_exit_code": decoder_command["exit_code"],
        "post_fix_expected_exit_code": 0,
    }
    write_json(R / "decoder_probe_comparison.json", payload)
    return payload


def write_environment(commands):
    status_code, status = run_git("status", "--short")
    head_code, head = run_git("rev-parse", "HEAD")
    branch_code, branch = run_git("rev-parse", "--abbrev-ref", "HEAD")
    uname = platform.uname()
    payload = {
        "schema": "fr02a.rework_02.environment.v1",
        "generated_at_utc": utc_now(),
        "worktree": str(ROOT),
        "cwd": os.getcwd(),
        "python_executable": sys.executable,
        "python_version": sys.version,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "uname": {
            "system": uname.system,
            "node": uname.node,
            "release": uname.release,
            "version": uname.version,
            "machine": uname.machine,
        },
        "cpu_count": os.cpu_count(),
        "math_nextafter_available": hasattr(math, "nextafter"),
        "git": {
            "head": head.strip() if head_code == 0 else None,
            "branch": branch.strip() if branch_code == 0 else None,
            "status_short": status.splitlines() if status_code == 0 else [],
        },
        "relevant_environment": {
            key: os.environ.get(key) for key in
            ("PATH", "PYTHONPATH", "OPENBLAS_NUM_THREADS",
             "PYTHONDONTWRITEBYTECODE", "LANG", "LC_ALL")
        },
        "suite_env_overrides": {
            item["name"]: item["env_overrides"] for item in commands
        },
    }
    write_json(FINAL / "environment.json", payload)
    write_json(R / "environment.json", payload)
    (FINAL / "git_status_final.txt").write_text(status, encoding="utf-8")
    (FINAL / "head_final.txt").write_text(head, encoding="utf-8")
    (FINAL / "branch_final.txt").write_text(branch, encoding="utf-8")
    diff_code, diff_stat = run_git("diff", "--stat")
    (FINAL / "git_diff_stat_final.txt").write_text(diff_stat,
                                                   encoding="utf-8")
    cached_code, cached_stat = run_git("diff", "--cached", "--stat")
    (FINAL / "git_diff_cached_final.stat").write_text(cached_stat,
                                                      encoding="utf-8")
    return payload


def write_scope(hashes, probe_payload, decoder_payload):
    comparisons = hashes["comparisons"]
    by_name = hashes["by_name"]
    changed = [item["relative_to_worktree"] for item in comparisons
               if item["changed"] and Path(item["path"]).name in
               {name for _, name in SHARED}]
    original_records = hashes["original_records"]
    original_unchanged = all(not item["changed"]
                             for item in original_records)
    original_script_ok = (
        sha256(ORIGINAL_PROBE_DIR / PROBE_SCRIPT_NAME)
        == sha256(PROBE / PROBE_SCRIPT_NAME)
        == sha256(IND_PRE_FIX / PROBE_SCRIPT_NAME)
        == sha256(IND_POST_FIX / PROBE_SCRIPT_NAME))
    original_json_ok = (
        sha256(ORIGINAL_PROBE_DIR / "independent_probes.json")
        == sha256(IND_ORIGINAL / "independent_probes.json")
        == sha256(IND_PRE_FIX / "independent_probes.json"))
    original_log_ok = (
        sha256(ORIGINAL_PROBE_DIR / "independent_probes.log")
        == sha256(IND_ORIGINAL / "independent_probes.log")
        == sha256(IND_PRE_FIX / "run.log"))
    decoder_script_ok = (
        sha256(ORIGINAL_PROBE_DIR / DECODER_SCRIPT_NAME)
        == sha256(DECODER_PROBE / DECODER_SCRIPT_NAME)
        == sha256(DEC_PRE_FIX / DECODER_SCRIPT_NAME)
        == sha256(DEC_POST_FIX / DECODER_SCRIPT_NAME))
    decoder_pre_json_ok = (
        sha256(ORIGINAL_PROBE_DIR / "decoder_time_probes.json")
        == sha256(DEC_ORIGINAL / "decoder_time_probes.json")
        == sha256(DEC_PRE_FIX / "decoder_time_probes.json"))
    decoder_pre_log_ok = (
        sha256(ORIGINAL_PROBE_DIR / "decoder_time_probes.log")
        == sha256(DEC_ORIGINAL / "decoder_time_probes.log")
        == sha256(DEC_PRE_FIX / "run.log"))
    status_code, status = run_git("status", "--short")
    tracked_code, tracked = run_git("diff", "--name-only")
    untracked_code, untracked = run_git("ls-files", "--others",
                                        "--exclude-standard")
    payload = {
        "schema": "fr02a.rework_02.scope_verification.v1",
        "generated_at_utc": utc_now(),
        "worktree": str(ROOT),
        "rework_changed_files": changed,
        "rework_changed_count": len(changed),
        "expected_rework_changed_files": sorted(EXPECTED_REWORK_CHANGED),
        "only_expected_rework_files_changed":
            {Path(item).name for item in changed} == EXPECTED_REWORK_CHANGED,
        "shared_files_unchanged": {
            "formation_geometry.py": not by_name[
                "formation_geometry.py"]["changed"],
            "formation_kinematics.py": not by_name[
                "formation_kinematics.py"]["changed"],
            "formation_retiming.py": not by_name[
                "formation_retiming.py"]["changed"],
            "uniform_timing.py": not by_name["uniform_timing.py"]["changed"],
            "test_uniform_timing.py": not by_name[
                "test_uniform_timing.py"]["changed"],
        },
        "fr01_unchanged": {
            "uniform_motion.py": not by_name[
                "uniform_motion.py"]["changed"],
            "test_uniform_motion.py": not by_name[
                "test_uniform_motion.py"]["changed"],
        },
        "original_docs_unchanged": {
            "REPORT.md": not by_name["REPORT.md"]["changed"],
            "CODEX_ACCEPTANCE.md": not by_name[
                "CODEX_ACCEPTANCE.md"]["changed"],
        },
        "original_evidence_unchanged": {
            "baseline": str((PRE / "original_hashes.sha256").relative_to(R)),
            "all_unchanged": original_unchanged,
            "changed_files": [item["relative_to_worktree"]
                              for item in original_records if item["changed"]],
            "file_count": len(original_records),
        },
        "original_codex_probe_evidence": {
            "directory": str(ORIGINAL_PROBE_DIR.relative_to(ROOT)),
            "script_matches_copied_original": original_script_ok,
            "json_matches_copied_original": original_json_ok,
            "log_matches_copied_original": original_log_ok,
            "script_sha256":
                sha256(ORIGINAL_PROBE_DIR / PROBE_SCRIPT_NAME),
            "json_sha256":
                sha256(ORIGINAL_PROBE_DIR / "independent_probes.json"),
            "log_sha256":
                sha256(ORIGINAL_PROBE_DIR / "independent_probes.log"),
        },
        "original_codex_decoder_probe_evidence": {
            "directory": str(ORIGINAL_PROBE_DIR.relative_to(ROOT)),
            "script_matches_copied_original": decoder_script_ok,
            "json_matches_copied_original": decoder_pre_json_ok,
            "log_matches_copied_original": decoder_pre_log_ok,
            "script_sha256":
                sha256(ORIGINAL_PROBE_DIR / DECODER_SCRIPT_NAME),
            "json_sha256":
                sha256(ORIGINAL_PROBE_DIR / "decoder_time_probes.json"),
            "log_sha256":
                sha256(ORIGINAL_PROBE_DIR / "decoder_time_probes.log"),
        },
        "rework_probe_outputs_new": {
            "independent": {
                "json": str((IND_POST_FIX / "independent_probes.json")
                            .relative_to(R)),
                "log": str((IND_POST_FIX / "independent_probes.log")
                           .relative_to(R)),
                "exit_code": str(
                    (IND_POST_FIX / "independent_probes.exit_code")
                    .relative_to(R)),
            },
            "decoder": {
                "json": str((DEC_POST_FIX / "decoder_time_probes.json")
                            .relative_to(R)),
                "log": str((DEC_POST_FIX / "decoder_time_probes.log")
                           .relative_to(R)),
                "exit_code": str(
                    (DEC_POST_FIX / "decoder_time_probes.exit_code")
                    .relative_to(R)),
            },
        },
        "probe_failures_eliminated": {
            "boundary": probe_payload["boundary_green_preserved"],
            "s_wheel": probe_payload["s_wheel_green_preserved"],
            "interior_preserved": probe_payload["interior_all_passed"],
            "decoder_pre_fix_failures_reproduced":
                decoder_payload["pre_fix_reproduces_two_failures"],
            "decoder_post_fix_rejections":
                decoder_payload["post_fix_both_rejected_at_decode"],
        },
        "git": {
            "head": run_git("rev-parse", "HEAD")[1].strip(),
            "branch": run_git("rev-parse", "--abbrev-ref", "HEAD")[1].strip(),
            "status_short": status.splitlines() if status_code == 0 else [],
            "tracked_modified": tracked.splitlines() if tracked_code == 0 else [],
            "expected_tracked_modified": list(EXPECTED_TRACKED_SHARED),
            "tracked_modified_only_expected": (
                sorted(tracked.splitlines()) == sorted(EXPECTED_TRACKED_SHARED)
                if tracked_code == 0 else False),
            "untracked": untracked.splitlines() if untracked_code == 0 else [],
        },
        "forbidden_actions": {
            "git_commit_made": False,
            "original_repository_modified": False,
            "original_fr02_evidence_overwritten": not original_unchanged,
            "fr02_b_entered": False,
            "ros_or_search_work_done": False,
        },
    }
    write_json(R / "scope_verification.json", payload)
    return payload


def write_self_check(regression, hashes, probe, decoder, scope, commands):
    r1 = read_json(ARTIFACTS / "R1_near_endpoint.json")
    r2 = read_json(ARTIFACTS / "R2_spin_wheel_vectors.json")
    r3 = read_json(ARTIFACTS / "R3_decoder_time_axis.json")
    checks = []

    def add(name, passed, detail):
        checks.append({"name": name, "passed": bool(passed),
                       "detail": detail})

    add("all_seven_suites_exit_zero",
        all(item["exit_code"] == 0 for item in regression["suites"]),
        {item["name"]: item["exit_code"] for item in regression["suites"]})
    add("all_seven_suites_expected_test_counts",
        all(item["ran_tests"] == item["expected_tests"]
            for item in regression["suites"]),
        {item["name"]: [item["ran_tests"], item["expected_tests"]]
         for item in regression["suites"]})
    add("regression_total_97",
        regression["total_tests"] == 97,
        {"total_tests": regression["total_tests"]})
    add("timing_results_A01_A05_passed",
        regression["results"]["timing"]["status"] == "PASSED"
        and regression["results"]["timing"]["all_cases_passed"],
        regression["results"]["timing"])
    add("trajectory_results_A06_A10_R1_R2_passed",
        regression["results"]["trajectory"]["status"] == "PASSED"
        and regression["results"]["trajectory"]["all_cases_passed"],
        regression["results"]["trajectory"])
    add("fr01_cases_all_passed",
        regression["results"]["fr01"]["all_cases_passed"],
        regression["results"]["fr01"])
    add("R1_no_progress_overshoot",
        r1["max_progress_overshoot"] <= 0.0,
        r1["max_progress_overshoot"])
    add("R1_progress_lower_bound",
        r1["min_progress"] >= 0.0,
        r1["min_progress"])
    add("R1_monotonic_near_endpoint",
        r1["max_monotonic_drop"] <= 1e-12,
        r1["max_monotonic_drop"])
    add("R1_roundtrip_finite_and_small",
        r1["max_roundtrip_abs_error"] <= 1e-7,
        r1["max_roundtrip_abs_error"])
    add("R1_center_and_rigidity_within_1e-7",
        r1["max_center_error_m"] <= 1e-7
        and r1["max_rigidity_error_m"] <= 1e-7,
        [r1["max_center_error_m"], r1["max_rigidity_error_m"]])
    add("R1_arc_and_projection_within_thresholds",
        r1["max_arc_qf_error"] <= 1e-9
        and r1["max_projection_state_error"] <= 1e-12
        and r1["max_setup_wheel_angle_overshoot_rad"] <= 1e-12,
        [r1["max_arc_qf_error"], r1["max_projection_state_error"],
         r1["max_setup_wheel_angle_overshoot_rad"]])
    add("R1_rounded_boundary_requests_recorded",
        r1["rounded_boundary_skipped_count"]
        == len(r1["skipped_boundary_requests"])
        and all(
            not (item["t0_s"] <= item["requested_time_s"]
                 < item["end_time_s"])
            for item in r1["skipped_boundary_requests"]),
        [r1["rounded_boundary_skipped_count"],
         len(r1["skipped_boundary_requests"]),
         r1["skipped_boundary_requests"]])
    add("R1_near_time_count_nonzero",
        r1["near_time_count"] >= 100,
        r1["near_time_count"])
    add("R2_vector_residual_within_1e-7",
        r2["max_vector_error_mps"] <= 1e-7,
        r2["max_vector_error_mps"])
    add("R2_peak_consistency_within_1e-12",
        r2["max_peak_wheel_speed_error_mps"] <= 1e-12,
        r2["max_peak_wheel_speed_error_mps"])
    add("R2_roundtrip_zero",
        r2["max_roundtrip_abs_error"] == 0.0,
        r2["max_roundtrip_abs_error"])
    add("R2_folded_and_unfolded_covered",
        r2["folded_wheel_samples"] > 0
        and r2["unfolded_wheel_samples"] > 0
        and len(r2["zero_yaw_vehicle_samples"]) > 0,
        [r2["folded_wheel_samples"], r2["unfolded_wheel_samples"],
         len(r2["zero_yaw_vehicle_samples"])])

    def finite_tree(value):
        if isinstance(value, bool) or value is None or isinstance(value, str):
            return True
        if isinstance(value, (int, float)):
            return math.isfinite(float(value))
        if isinstance(value, dict):
            return all(finite_tree(item) for item in value.values())
        if isinstance(value, (list, tuple)):
            return all(finite_tree(item) for item in value)
        return False

    gaps = r3["gap_overlap_cases"]
    collapsed = r3["collapsed_cases"]
    duration_mismatch = r3["duration_mismatch_cases"]
    accepted = r3["support_boundary_acceptance"]
    unsupported = r3["support_boundary_rejection"]
    hold = r3["legal_zero_motion_hold"]
    normals = r3["normal_roundtrips"]

    add("R3_within_or_at_old_tolerance_gaps_and_overlaps_rejected",
        len(gaps) == 8
        and all(item["decode_reason"] == "PHASE_TIME_DISCONTINUITY"
                and item["within_old_tolerance"]
                and item["actual_delta_s"] != 0.0
                for item in gaps)
        and any(item["actual_delta_s"] > 0.0 for item in gaps)
        and any(item["actual_delta_s"] < 0.0 for item in gaps)
        and any(item["actual_delta_s"] == 1e-9 for item in gaps)
        and any(item["actual_delta_s"] == -1e-9 for item in gaps),
        gaps)
    add("R3_collapsed_positive_interval_rejected",
        len(collapsed) == 1
        and all(item["decode_reason"] == "PHASE_TIME_UNREPRESENTABLE"
                and item["collapsed_to_t0"]
                and item["stored_duration_s"] > 0.0
                for item in collapsed),
        collapsed)
    add("R3_total_duration_final_boundary_mismatch_rejected",
        len(duration_mismatch) == 3
        and all(item["decode_reason"] == "PLAN_DURATION_MISMATCH"
                and item["delta_s"] != 0.0
                for item in duration_mismatch),
        duration_mismatch)
    add("R3_numeric_support_boundaries_accepted_and_finite",
        len(accepted) == 2
        and {item["duration_s"] for item in accepted} == {1e-100, 1e100}
        and all(item["terminal_progress_rad"]
                == item["expected_progress_rad"]
                and finite_tree(item["states"])
                for item in accepted),
        accepted)
    add("R3_unsupported_numeric_clocks_rejected",
        len(unsupported) == 4
        and all(item["decode_reason"] == "PHASE_DURATION_UNSUPPORTED"
                for item in unsupported),
        unsupported)
    add("R3_legal_zero_motion_hold_preserved",
        hold["duration_s"] == 0.0
        and hold["phase_count"] == 0
        and hold["sample_times_s"] == [0.0]
        and hold["state"]["stationary"]
        and hold["state"]["progress"] == 0.0
        and hold["state"]["progress_rate"] == 0.0,
        {key: hold[key] for key in
         ("duration_s", "phase_count", "sample_times_s")})
    add("R3_normal_roundtrip_and_terminal_arrival",
        len(normals) == 3
        and all(
            item["last_motion_end_s"] == item["duration_s"]
            and item["terminal_progress_rad"]
            == item["terminal_expected_progress_rad"]
            and item["max_roundtrip_abs_error"] <= 1e-10
            and item["phase_boundaries_s"][0] == 0.0
            and all(item["phase_boundaries_s"][index]
                    == item["phase_end_times_s"][index - 1]
                    for index in range(1,
                                       len(item["phase_boundaries_s"])))
            and item["phase_end_times_s"][-1] == item["duration_s"]
            for item in normals),
        normals)
    add("probe_exit_zero",
        probe["rework_exit_code"] == 0,
        probe["rework_exit_code"])
    add("probe_no_boundary_failures",
        probe["rework"]["boundary_failed_row_count"] == 0
        and probe["rework"]["explicit_boundary_failure_count"] == 0,
        probe["rework"]["boundary_failed_row_count"])
    add("probe_no_S_wheel_failures",
        probe["rework"]["s_wheel_failed_row_count"] == 0
        and probe["rework"]["explicit_S_wheel_failure_count"] == 0,
        probe["rework"]["s_wheel_failed_row_count"])
    add("probe_66_interior_states_preserved",
        probe["interior_all_passed"],
        probe["rework"]["interior_row_count"])
    add("probe_71_boundary_and_12_wheel_rows_still_green",
        probe["boundary_green_preserved"]
        and probe["s_wheel_green_preserved"],
        {
            "original": {
                "boundary_rows": probe["original"]["boundary_row_count"],
                "boundary_failures":
                    probe["original"]["boundary_failed_row_count"],
                "s_wheel_rows": probe["original"]["s_wheel_row_count"],
                "s_wheel_failures":
                    probe["original"]["s_wheel_failed_row_count"],
            },
            "rework": {
                "boundary_rows": probe["rework"]["boundary_row_count"],
                "boundary_failures":
                    probe["rework"]["boundary_failed_row_count"],
                "s_wheel_rows": probe["rework"]["s_wheel_row_count"],
                "s_wheel_failures":
                    probe["rework"]["s_wheel_failed_row_count"],
            },
        })
    add("decoder_probe_exit_zero",
        decoder["post_fix_exit_code"] == 0
        and decoder["post_fix_expected_exit_code"] == 0,
        decoder["post_fix_exit_code"])
    add("decoder_probe_pre_fix_two_failures_reproduced",
        decoder["pre_fix_reproduces_two_failures"]
        and decoder["pre_fix_exit_code"] == 1,
        {"pre_fix": decoder["pre_fix"],
         "exit_code": decoder["pre_fix_exit_code"]})
    add("decoder_probe_post_fix_both_rejected_at_decode",
        decoder["post_fix_both_rejected_at_decode"],
        {"post_fix": decoder["post_fix"],
         "expected_reasons": decoder["expected_decode_reasons"]})
    add("probe_original_evidence_not_overwritten",
        scope["original_codex_probe_evidence"][
            "script_matches_copied_original"]
        and scope["original_codex_probe_evidence"][
            "json_matches_copied_original"]
        and scope["original_codex_probe_evidence"][
            "log_matches_copied_original"]
        and scope["original_codex_decoder_probe_evidence"][
            "script_matches_copied_original"]
        and scope["original_codex_decoder_probe_evidence"][
            "json_matches_copied_original"]
        and scope["original_codex_decoder_probe_evidence"][
            "log_matches_copied_original"],
        {
            "independent": scope["original_codex_probe_evidence"],
            "decoder": scope["original_codex_decoder_probe_evidence"],
        })
    add("original_evidence_and_prior_rework_unchanged",
        scope["original_evidence_unchanged"]["all_unchanged"],
        scope["original_evidence_unchanged"])
    add("scope_only_two_rework_files_changed",
        scope["only_expected_rework_files_changed"]
        and scope["rework_changed_count"] == 2,
        scope["rework_changed_files"])
    add("git_tracked_changes_only_three_shared_adaptations",
        scope["git"]["tracked_modified_only_expected"],
        scope["git"]["tracked_modified"])
    add("scope_shared_fr01_and_timing_unchanged",
        all(scope["shared_files_unchanged"].values())
        and all(scope["fr01_unchanged"].values())
        and all(scope["original_docs_unchanged"].values()),
        {
            "shared": scope["shared_files_unchanged"],
            "fr01": scope["fr01_unchanged"],
            "docs": scope["original_docs_unchanged"],
        })
    add("all_commands_recorded_exit_zero",
        all(item["exit_code"] == 0
            and item["exit_code"] == item["expected_exit_code"]
            for item in commands),
        {item["name"]: item["exit_code"] for item in commands})
    add("final_copies_match_current_sources",
        all(
            sha256(FINAL / "shared" / name) == sha256(source)
            for source, name in SHARED),
        "sha256")
    add("no_git_commit_made",
        scope["git"]["head"]
        == (PRE / "head.txt").read_text(encoding="utf-8").strip(),
        {"head": scope["git"]["head"],
         "pre_rework_head":
             (PRE / "head.txt").read_text(encoding="utf-8").strip()})

    failures = [item for item in checks if not item["passed"]]
    payload = {
        "schema": "fr02a.rework_02.self_check.v1",
        "generated_at_utc": utc_now(),
        "overall": "PASSED" if not failures else "FAILED",
        "checks": checks,
        "failures": failures,
    }
    write_json(CHECKS / "self_check.json", payload)
    return payload


def write_manifest():
    entries = []
    for path in sorted(R.rglob("*")):
        if not path.is_file():
            continue
        if path.name == "evidence_manifest.json":
            continue
        entries.append({
            "path": str(path.relative_to(R)),
            "size_bytes": path.stat().st_size,
            "sha256": sha256(path),
        })
    payload = {
        "schema": "fr02a.rework_02.evidence_manifest.v1",
        "generated_at_utc": utc_now(),
        "root": str(R),
        "excludes": ["evidence_manifest.json"],
        "file_count": len(entries),
        "total_bytes": sum(item["size_bytes"] for item in entries),
        "files": entries,
    }
    write_json(R / "evidence_manifest.json", payload)
    return payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest-only", action="store_true")
    args = parser.parse_args()

    if args.manifest_only:
        payload = write_manifest()
        print("refreshed {} evidence files".format(payload["file_count"]))
        return 0

    LOGS.mkdir(parents=True, exist_ok=True)
    RESULTS.mkdir(parents=True, exist_ok=True)
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    FINAL.mkdir(parents=True, exist_ok=True)
    DIFF.mkdir(parents=True, exist_ok=True)
    CHECKS.mkdir(parents=True, exist_ok=True)

    commands = []
    for suite in SUITES:
        name = suite["name"]
        commands.append(run_command(
            name, suite["argv"], suite["env"],
            LOGS / "{}.log".format(name),
            LOGS / "{}.exit_code".format(name)))
    probe_command = run_command(
        "codex_probe", (str(IND_POST_FIX / PROBE_SCRIPT_NAME),), {},
        IND_POST_FIX / "independent_probes.log",
        IND_POST_FIX / "independent_probes.exit_code")
    commands.append(probe_command)
    decoder_command = run_command(
        "codex_decoder_probe",
        (str(DEC_POST_FIX / DECODER_SCRIPT_NAME),), {},
        DEC_POST_FIX / "decoder_time_probes.log",
        DEC_POST_FIX / "decoder_time_probes.exit_code")
    commands.append(decoder_command)

    command_manifest = {
        "schema": "fr02a.rework_02.command_manifest.v1",
        "generated_at_utc": utc_now(),
        "cwd": str(ROOT),
        "python_executable": sys.executable,
        "common_env": {"OPENBLAS_NUM_THREADS": "1",
                       "PYTHONDONTWRITEBYTECODE": "1"},
        "commands": commands,
        "diff_commands": write_diffs(),
        "environment": "final/environment.json and environment.json",
    }
    write_json(R / "command_manifest.json", command_manifest)

    copy_final_files()
    hashes = write_hashes()
    regression = collect_regression(commands)
    probe_payload = write_probe_comparison(probe_command)
    decoder_payload = write_decoder_probe_comparison(decoder_command)
    write_environment(commands)
    scope = write_scope(hashes, probe_payload, decoder_payload)
    self_check = write_self_check(regression, hashes, probe_payload,
                                  decoder_payload, scope, commands)
    manifest = write_manifest()

    print(json.dumps({
        "self_check": self_check["overall"],
        "regression_total_tests": regression["total_tests"],
        "probe_exit": probe_command["exit_code"],
        "probe_boundary_failures":
            probe_payload["rework"]["boundary_failed_row_count"],
        "probe_s_wheel_failures":
            probe_payload["rework"]["s_wheel_failed_row_count"],
        "decoder_probe_exit": decoder_command["exit_code"],
        "decoder_post_fix_reasons": [
            item["reason"] for item in decoder_payload["post_fix"]["cases"]],
        "evidence_files": manifest["file_count"],
    }, indent=2))
    return 0 if self_check["overall"] == "PASSED" else 1


if __name__ == "__main__":
    sys.exit(main())