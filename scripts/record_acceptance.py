#!/usr/bin/env python3
"""Run one acceptance command with reproducible provenance and private output."""

import argparse
from datetime import datetime, timezone
from hashlib import sha256
import json
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
from time import monotonic


ROOT = Path(__file__).resolve().parents[1]


def command_output(*argv):
    try:
        result = subprocess.run(argv, cwd=ROOT, text=True, capture_output=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def read_optional(path):
    try:
        return path.read_text().strip()
    except OSError:
        return None


def save_manifest(directory, manifest):
    temporary = directory / ".manifest.tmp"
    temporary.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    temporary.replace(directory / "manifest.json")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input", action="append", type=Path, default=[],
                        help="Save and hash a configuration file used by the command.")
    args, command = parser.parse_known_args()
    if len(command) < 2 or command[0] != "--":
        parser.error("pass the command after --")
    output = args.output.resolve()
    for path in args.input:
        if not path.is_file():
            parser.error(f"input file missing: {path}")
    output.mkdir(mode=0o700, parents=True, exist_ok=False)

    governors = sorted({value for path in Path("/sys/devices/system/cpu").glob(
        "cpu[0-9]*/cpufreq/scaling_governor") if (value := read_optional(path)) is not None})
    manifest = {
        "schema": 1,
        "status": "running",
        "started_at": datetime.now(timezone.utc).isoformat(),
        "source_sha": command_output("git", "rev-parse", "HEAD"),
        "source_tree": command_output("git", "rev-parse", "HEAD^{tree}"),
        "worktree_status": command_output("git", "status", "--porcelain=v1"),
        "command": command[1:],
        "cwd": str(ROOT),
        "platform": platform.platform(),
        "os_release": read_optional(Path("/etc/os-release")),
        "cpu": next((line.split(":", 1)[1].strip() for line in
                     (read_optional(Path("/proc/cpuinfo")) or "").splitlines()
                     if line.startswith("model name")), None),
        "cpu_governors": governors,
        "cpu_affinity": sorted(os.sched_getaffinity(0)),
        "memory_kib": os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE") // 1024,
        "python": sys.version,
        "compiler": command_output("c++", "--version"),
        "ros_distro": os.environ.get("ROS_DISTRO", "humble"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "gazebo": command_output("gz", "sim", "--versions"),
        "dependencies": command_output("dpkg-query", "-W", "ros-humble-rcl", "ros-humble-rclcpp",
                                       "ros-humble-rmw-fastrtps-cpp", "ros-humble-ros-gzharmonic",
                                       "libgz-sim8", "python3-numpy", "python3-scipy"),
        "inputs": [],
    }
    if manifest["worktree_status"]:
        patch = subprocess.check_output(("git", "diff", "--binary", "HEAD"), cwd=ROOT)
        (output / "working.patch").write_bytes(patch)
        manifest["working_patch_sha256"] = sha256(patch).hexdigest()
    if args.input:
        (output / "inputs").mkdir()
    for index, path in enumerate(args.input, start=1):
        data = path.read_bytes()
        saved = Path("inputs") / f"{index:02d}-{path.name}"
        (output / saved).write_bytes(data)
        manifest["inputs"].append({"path": str(path.resolve()), "saved": str(saved),
                                   "sha256": sha256(data).hexdigest()})
    save_manifest(output, manifest)
    print(f"Acceptance evidence: {output}", flush=True)
    print(shlex.join(command[1:]), flush=True)
    started = monotonic()
    code = 1
    try:
        with (output / "command.log").open("wb") as log:
            process = subprocess.Popen(
                command[1:], cwd=ROOT, env={**os.environ, "COPTRANS_EVIDENCE_DIR": str(output)},
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            )
            try:
                for line in process.stdout:
                    log.write(line)
                    log.flush()
                    sys.stdout.buffer.write(line)
                    sys.stdout.buffer.flush()
            finally:
                process.stdout.close()
                code = process.wait()
    finally:
        manifest.update(status="passed" if code == 0 else "failed", exit_code=code,
                        elapsed_s=monotonic() - started,
                        ended_at=datetime.now(timezone.utc).isoformat())
        save_manifest(output, manifest)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
