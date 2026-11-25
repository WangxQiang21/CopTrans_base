#!/usr/bin/env python3
"""Run one CopTrans case with monotonic logs and a read-only timing subscriber."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
from threading import Event, Thread
from time import monotonic_ns

import psutil


PROCESS_START = re.compile(r"\[([^\]]+)\]: process started with pid \[(\d+)\]")
SESSION = re.compile(r"ROS_DOMAIN_ID=(\d+)\s+GZ_PARTITION=(\S+)")
HERE = Path(__file__).resolve().parent


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_environment(root, python):
    return [
        "bash", "-c", 'source /opt/ros/humble/setup.bash && source "$1" && shift && exec "$@"',
        "coptrans-observer", str(root / "ros2_ws/install/setup.bash"), python,
    ]


def capture_metadata(root):
    def git(*args):
        return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()

    def result(*args):
        process = subprocess.run(args, capture_output=True, text=True)
        return process.stdout.strip() if process.returncode == 0 else None

    stdout = result("dpkg-query", "-W", "g++", "ros-humble-rclpy",
                    "ros-humble-rmw-fastrtps-cpp", "gz-harmonic", "libgz-sim8")
    cache = root / "ros2_ws/build/fwis_backends/CMakeCache.txt"
    configuration = root / "ros2_ws/install/fwis_control/share/fwis_control/config/ranger_tracking.yaml"
    cpu = Path("/proc/cpuinfo").read_text()
    os_release = Path("/etc/os-release").read_text()
    return {
        "git_branch": git("branch", "--show-current"),
        "git_sha": git("rev-parse", "HEAD"),
        "worktree": git("status", "--porcelain"),
        "local_remote_tracking_left_right": git("rev-list", "--left-right", "--count",
                                                "HEAD...origin/ros2-humble").split(),
        "cpu": re.search(r"^model name\s*:\s*(.*)$", cpu, re.MULTILINE).group(1),
        "logical_cpu_count": psutil.cpu_count(),
        "physical_cpu_count": psutil.cpu_count(logical=False),
        "memory_bytes": psutil.virtual_memory().total,
        "cpu_governors": sorted({path.read_text().strip() for path in
                                  Path("/sys/devices/system/cpu").glob("cpu*/cpufreq/scaling_governor")}),
        "operating_system": re.search(r"^PRETTY_NAME=(.*)$", os_release, re.MULTILINE).group(1).strip('"'),
        "compiler": result("c++", "-dumpfullversion"),
        "installed_versions": stdout.splitlines() if stdout else [],
        "build_type": (re.search(r"^CMAKE_BUILD_TYPE:STRING=(.*)$", cache.read_text(), re.MULTILINE).group(1)
                       if cache.exists() else None),
        "controller_configuration_sha256": sha256(configuration),
    }


def run(root, task, scenario, experiment, controller):
    # The existing launcher owns its DDS domain, software config and run directory.
    command = [str(root / "scripts/test.sh"), "--task", task, "--scenario", scenario,
               "--experiment", experiment, "--controller-backend", controller, "--no-build"]
    cli_help = subprocess.run([command[0], "--help"], cwd=root, capture_output=True, text=True, check=True)
    if "--speed-profile" in cli_help.stdout:
        command.extend(("--speed-profile", "baseline"))
    environment = capture_metadata(root)
    started = monotonic_ns()
    with subprocess.Popen(command, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, bufsize=1) as process:
        root_process = psutil.Process(process.pid)
        labels, peaks = {}, {}
        stop = Event()
        artifact_dir = None
        observer = None
        launch_events = None
        observer_output = None

        def sample():
            while not stop.wait(0.2):
                try:
                    descendants = root_process.children(recursive=True)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    break
                for child in descendants:
                    try:
                        cpu = child.cpu_times()
                        label = labels.get(child.pid)
                        if label is None:
                            label = next((labels[parent.pid] for parent in child.parents()
                                          if parent.pid in labels), child.name())
                        stat = peaks.setdefault(child.pid, {"cpu_s": 0.0, "peak_rss_bytes": 0})
                        stat["label"] = label
                        stat["cpu_s"] = max(stat["cpu_s"], cpu.user + cpu.system)
                        stat["peak_rss_bytes"] = max(stat["peak_rss_bytes"], child.memory_info().rss)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue

        sampler = Thread(target=sample, daemon=True)
        sampler.start()
        try:
            for line in process.stdout:
                line = line.rstrip()
                stamp = monotonic_ns()
                if artifact_dir is None and line.startswith("Run directory "):
                    artifact_dir = Path(line.removeprefix("Run directory "))
                    launch_events = (artifact_dir / "launch_events.jsonl").open("w", encoding="utf-8")
                    print(f"{(stamp - started) / 1e9:.3f}s  {line}", flush=True)
                if launch_events is not None:
                    launch_events.write(json.dumps({"observed_monotonic_ns": stamp, "text": line},
                                                   ensure_ascii=False) + "\n")
                    launch_events.flush()
                child = PROCESS_START.search(line)
                if child is not None:
                    labels[int(child[2])] = child[1]
                domain = SESSION.fullmatch(line)
                if domain is not None and artifact_dir is not None:
                    obs_cmd = [*source_environment(root, "/usr/bin/python3"), str(HERE / "observe.py"),
                               str(artifact_dir), task, str(artifact_dir / "dds_trace.jsonl")]
                    obs_env = {**os.environ, "ROS_DOMAIN_ID": domain[1], "ROS_LOCALHOST_ONLY": "1",
                               "GZ_PARTITION": domain[2], "ROS_LOG_DIR": str(artifact_dir / "ros")}
                    observer_output = (artifact_dir / "observer.log").open("w", encoding="utf-8")
                    observer = subprocess.Popen(obs_cmd, cwd=root, env=obs_env, stdout=observer_output,
                                                stderr=subprocess.STDOUT)
                if line.startswith("Result "):
                    print(f"{(stamp - started) / 1e9:.3f}s  {line}", flush=True)
            code = process.wait()
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT)
            code = process.wait()
        finally:
            stop.set()
            sampler.join()
            if observer is not None:
                observer.send_signal(signal.SIGINT)
                try:
                    observer.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    observer.terminate()
                    observer.wait()
            if observer_output is not None:
                observer_output.close()
            if launch_events is not None:
                launch_events.close()
        finished = monotonic_ns()
    if artifact_dir is not None:
        capture = {
            "trace_id": artifact_dir.name,
            "command": command,
            "started_monotonic_ns": started,
            "wall_s": (finished - started) / 1e9,
            "exit_code": code,
            "environment": environment,
            "sample_interval_s": 0.2,
            "processes": list(peaks.values()),
        }
        (artifact_dir / "capture.json").write_text(json.dumps(capture, indent=2) + "\n")
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=HERE.parents[1])
    parser.add_argument("--task", choices=("single", "multi"), required=True)
    parser.add_argument("--scenario", default="factory_indoor_01")
    parser.add_argument("--experiment", choices=("short_turn", "short_straight", "full_route"), required=True)
    parser.add_argument("--controller", choices=("nonlinear_feedback", "pid"), default="nonlinear_feedback")
    args = parser.parse_args()
    raise SystemExit(run(args.root.resolve(), args.task, args.scenario, args.experiment, args.controller))


if __name__ == "__main__":
    main()
