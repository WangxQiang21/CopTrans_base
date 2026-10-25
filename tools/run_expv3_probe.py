#!/usr/bin/env python3
"""Run one reproducible expv3 ROS probe on an isolated ROS master.

The case manifest supplies identical start/goal values to the planner input
and independent swept-footprint checker. All run inputs, logs, hashes, and
results are kept under relust/probes/ (or --output-dir).
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import shutil
import socket
import subprocess
import sys
import time

import numpy as np
from PIL import Image
import yaml

from scale_map_runtime import scale_map


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = Path(__file__).with_name("expv3_eval_cases.yaml")
RESERVED_PORTS = {11311, 11312, 11320}
DEFAULT_PORT = 11453
OVERLAY = ROOT / "catkin_ws"
PARAMS = ROOT / "catkin_ws/src/fwis_v3_demo/config/params.yaml"
PLUGIN_DIR = OVERLAY / "devel/lib/fwis_v3_optimizer/casadi_plugins"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def tree_hash(path: Path) -> tuple[str, list[dict[str, str]]]:
    entries = []
    digest = hashlib.sha256()
    for file_path in sorted(item for item in path.rglob("*") if item.is_file()):
        relative = file_path.relative_to(path).as_posix()
        file_hash = sha256_file(file_path)
        entries.append({"path": relative, "sha256": file_hash})
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_hash.encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest(), entries


def finite_pose(value, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{label} must contain x, y, yaw")
    pose = [float(item) for item in value]
    if not all(math.isfinite(item) for item in pose):
        raise ValueError(f"{label} contains a non-finite value")
    return pose


def validate_case(case_name: str, case: dict) -> None:
    for key in ("map_dir", "map_scale", "scene", "start", "goal",
                "xy_tol", "yaw_tol"):
        if key not in case:
            raise ValueError(f"case {case_name} is missing {key}")
    map_dir = (ROOT / case["map_dir"]).resolve()
    if not (map_dir / "map.yaml").is_file():
        raise FileNotFoundError(f"case map is missing map.yaml: {map_dir}")
    scale = float(case["map_scale"])
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError(f"case {case_name} has invalid map_scale")
    finite_pose(case["start"], f"case {case_name} start")
    finite_pose(case["goal"], f"case {case_name} goal")
    if (not math.isfinite(float(case["xy_tol"])) or
            float(case["xy_tol"]) <= 0 or
            not math.isfinite(float(case["yaw_tol"])) or
            float(case["yaw_tol"]) <= 0):
        raise ValueError(f"case {case_name} has invalid terminal tolerances")


def set_existing(mapping: dict, dotted_key: str, value) -> None:
    keys = dotted_key.split(".")
    current = mapping
    for key in keys[:-1]:
        if key not in current or not isinstance(current[key], dict):
            raise ValueError(f"unknown config path: {dotted_key}")
        current = current[key]
    if keys[-1] not in current:
        raise ValueError(f"unknown config path: {dotted_key}")
    current[keys[-1]] = value


def parse_overrides(values: list[str]) -> list[tuple[str, object]]:
    parsed = []
    for item in values:
        if "=" not in item:
            raise ValueError(f"--set must be PATH=VALUE: {item}")
        key, raw = item.split("=", 1)
        if not key.strip():
            raise ValueError(f"--set has an empty path: {item}")
        parsed.append((key.strip(), yaml.safe_load(raw)))
    return parsed


def map_grid_signature(map_dir: Path, config: dict) -> dict:
    with (map_dir / "map.yaml").open("r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)
    image_path = (map_dir / meta["image"]).resolve()
    if map_dir.resolve() not in image_path.parents:
        raise ValueError("map image path must stay within the map directory")
    image = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
    image = np.flipud(image)
    free_thresh = float(meta.get("free_thresh", 0.196)) * 255.0
    # The optimizer treats all non-free OccupancyGrid cells as occupied,
    # including unknown grayscale cells.
    occupied = np.asarray(image < 255.0 - free_thresh, dtype=np.uint8)
    vehicle = config["vehicle"]
    margin = float(config["collision"]["margin"])
    length, width = float(vehicle["length"]), float(vehicle["width"])
    return {
        "sha256": hashlib.sha256(occupied.tobytes()).hexdigest(),
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "resolution": float(np.float32(meta["resolution"])),
        "origin": [float(meta["origin"][0]), float(meta["origin"][1])],
        "half_vehicle": [(length + margin) / 2.0,
                         (width + margin) / 2.0],
    }


def validate_replay_seed(seed_path: Path, expected_map: dict,
                         start: list[float], goal: list[float]) -> dict:
    with seed_path.open("r", encoding="utf-8") as stream:
        record = json.load(stream)
    if record.get("schema") != "fwis_v3_joint_seed_v1":
        raise ValueError("unsupported frozen seed schema")
    saved_map = record.get("map", {})
    if saved_map.get("sha256") != expected_map["sha256"]:
        raise ValueError("frozen seed occupancy-grid signature mismatch")
    if (saved_map.get("width") != expected_map["width"] or
            saved_map.get("height") != expected_map["height"] or
            not math.isclose(float(saved_map.get("resolution", float("nan"))),
                             expected_map["resolution"], abs_tol=1e-7) or
            len(saved_map.get("origin", [])) != 2 or
            any(abs(float(a) - b) > 1e-9
                for a, b in zip(saved_map["origin"], expected_map["origin"])) or
            len(saved_map.get("half_vehicle", [])) != 2 or
            any(abs(float(a) - b) > 1e-9 for a, b in
                zip(saved_map["half_vehicle"], expected_map["half_vehicle"]))):
        raise ValueError("frozen seed map geometry signature mismatch")
    for key, target in (("start", start), ("goal", goal)):
        saved = record.get(key)
        if (not isinstance(saved, list) or len(saved) != 3 or
                any(abs(float(a) - b) > 1e-7
                    for a, b in zip(saved, target))):
            raise ValueError(f"frozen seed {key} does not match the selected case")
    return record


def assert_port_available(port: int) -> None:
    if port in RESERVED_PORTS:
        raise ValueError(f"ROS master port {port} is protected")
    if port < 1024 or port > 65535:
        raise ValueError("ROS master port must be in 1024..65535")
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        if not port_is_listening(port):
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", port))
    except OSError as exc:
        raise RuntimeError(f"ROS master port {port} is already occupied") from exc
    finally:
        probe.close()


def ros_shell_prefix(port: int) -> str:
    required = [Path("/opt/ros/noetic/setup.bash"),
                OVERLAY / "devel/setup.bash"]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing ROS setup file(s): " + ", ".join(missing))
    pieces = [
        "set -e",
        "unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH",
        "source " + shlex.quote(str(required[0])),
        "source " + shlex.quote(str(required[1])),
        "export ROS_PACKAGE_PATH=" + shlex.quote(str(OVERLAY / "src")) +
        ":${ROS_PACKAGE_PATH:-}",
        "export CMAKE_PREFIX_PATH=" + shlex.quote(str(OVERLAY / "devel")) +
        ":${CMAKE_PREFIX_PATH:-}",
        "export PYTHONPATH=" + shlex.quote(str(OVERLAY / "devel/lib/python3/dist-packages")) +
        ":${PYTHONPATH:-}",
        "export LD_LIBRARY_PATH=" + shlex.quote(str(OVERLAY / "devel/lib")) +
        ":${LD_LIBRARY_PATH:-}",
        f"export ROS_MASTER_URI=http://127.0.0.1:{port}",
        "export ROS_IP=127.0.0.1",
    ]
    if (PLUGIN_DIR / "libcasadi_nlpsol_bonminseed.so").is_file():
        pieces.append("export CASADIPATH=" + shlex.quote(str(PLUGIN_DIR)) +
                      ":${CASADIPATH:-}")
    return "; ".join(pieces)


def start_logged(command: list[str], log_path: Path,
                 prefix: str) -> tuple[subprocess.Popen, object]:
    log_stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", prefix + "; exec " + shlex.join(command)],
        cwd=ROOT, text=True, stdout=log_stream, stderr=subprocess.STDOUT,
        start_new_session=True)
    return process, log_stream


def terminate_process(process: subprocess.Popen | None,
                      grace_seconds: float = 6.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=grace_seconds)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=3.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        pass


def wait_for_master(port: int, process: subprocess.Popen,
                    timeout: float = 15.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"roscore exited early with code {process.returncode}")
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.settimeout(.2)
        try:
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                return
        finally:
            probe.close()
        time.sleep(.1)
    raise TimeoutError(f"roscore did not listen on port {port}")


def port_is_listening(port: int) -> bool:
    if port < 1 or port > 65535:
        return True
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.settimeout(.2)
    try:
        return probe.connect_ex(("127.0.0.1", port)) == 0
    except (OSError, OverflowError):
        return False
    finally:
        probe.close()


def process_running(pid: int) -> bool:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False
    # A zombie has exited and is waiting for its parent to reap it.
    closing = stat.rfind(")")
    return closing < 0 or stat[closing + 2:closing + 3] != "Z"


def project_ros_processes(master_uri: str) -> dict[int, str]:
    markers = (
        "optimizer_node.py", "fwis_v3_planner_node", "playback.py",
        "map_publisher.py", "scene_hints.py", "parking_lines.py",
    )
    processes = {}
    proc_root = Path("/proc")
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
            environment = (entry / "environ").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        command = raw.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
        if (master_uri.encode("ascii") in environment and command and
                any(marker in command for marker in markers) and
                str(ROOT) in command):
            processes[int(entry.name)] = command
    return processes


def process_matches(pid: int, expected_command: str) -> bool:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return False
    command = raw.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
    return command == expected_command


def stop_launch_children(prefix: str, children: dict[int, str],
                         shutdown_log_path: Path) -> list[int]:
    if not children:
        return []
    with shutdown_log_path.open("w", encoding="utf-8") as stream:
        try:
            completed = subprocess.run(
                ["bash", "-lc", prefix + "; exec rosnode kill -a"],
                cwd=ROOT, text=True, stdout=stream, stderr=subprocess.STDOUT,
                timeout=10.0, check=False, start_new_session=True)
            stream.write(f"\nrosnode_kill_exit={completed.returncode}\n")
        except subprocess.TimeoutExpired:
            stream.write("\nrosnode_kill_timeout=true\n")
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and any(
            process_running(pid) and process_matches(pid, command)
            for pid, command in children.items()):
        time.sleep(.1)
    remaining = [pid for pid, command in children.items()
                 if process_running(pid) and process_matches(pid, command)]
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and any(
            process_running(pid) and process_matches(pid, children[pid])
            for pid in remaining):
        time.sleep(.1)
    remaining = [pid for pid in remaining
                 if process_running(pid) and process_matches(pid, children[pid])]
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and any(
            process_running(pid) and process_matches(pid, children[pid])
            for pid in remaining):
        time.sleep(.1)
    remaining = [pid for pid in remaining
                 if process_running(pid) and process_matches(pid, children[pid])]
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and any(
            process_running(pid) and process_matches(pid, children[pid])
            for pid in remaining):
        time.sleep(.05)
    return [pid for pid in remaining
            if process_running(pid) and process_matches(pid, children[pid])]


def atomic_json(path: Path, data: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(json_safe(data), stream, ensure_ascii=False, indent=2,
                  sort_keys=False, allow_nan=False)
        stream.write("\n")
    os.replace(temporary, path)


def json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def finite_number(raw: str):
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def parse_optimizer_audits(log_path: Path) -> list[dict]:
    if not log_path.is_file():
        return []
    records = []
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "joint candidate solver=" not in line:
            continue
        values = {}
        fields = {
            "solver": r"\bsolver=([^ ]+)",
            "candidate_common_score": r"\bscore=([-+0-9.eE]+)",
            "model_objective": r"\bmodel_obj=([-+0-9.eE]+)",
            "seed_model_objective": r"\bseed_model_obj=([-+0-9.eE]+)",
            "seed_common_score": r"\bseed_score=([-+0-9.eE]+)",
            "seed_source": r"\bseed_source=([^ ]+)",
            "mode_word": r"\bmodes=([^ ]+)",
            "bonmin_status": r"\bbonmin_status=([^ ]+)",
            "model_objective_recomputed": r"\bmodel_obj_recomputed=([-+0-9.eE]+)",
            "model_objective_delta": r"\bmodel_obj_delta=([-+0-9.eE]+)",
        }
        numeric_keys = {
            "candidate_common_score", "model_objective",
            "seed_model_objective", "seed_common_score",
            "model_objective_recomputed", "model_objective_delta",
        }
        for key, pattern in fields.items():
            match = re.search(pattern, line)
            if match:
                raw = match.group(1)
                values[key] = finite_number(raw) if key in numeric_keys else raw
        residual = re.search(
            r"model_residual=\{count:([^,]+),max:([^}]+)\}", line)
        if residual:
            try:
                residual_count = int(residual.group(1))
            except ValueError:
                residual_count = None
            values["model_residual"] = {
                "count": residual_count,
                "max": finite_number(residual.group(2)),
            }
        timing = re.search(
            r"timing_s=\{bonmin:([0-9.]+),warm:([0-9.]+),total:([0-9.]+)\}",
            line)
        if timing:
            values["timing_s"] = {
                "bonmin": float(timing.group(1)),
                "warm_start": float(timing.group(2)),
                "joint_total": float(timing.group(3)),
            }
        terms = re.search(r"\bmodel_terms=(\{.*\})", line)
        if terms:
            try:
                values["model_objective_components"] = json_safe(
                    json.loads(terms.group(1)))
            except json.JSONDecodeError:
                values["model_objective_components_parse_error"] = True
        values["raw_log_line"] = line
        records.append(values)
    return records


def summarize_probe(data: dict | None, audits: list[dict] | None = None) -> dict | None:
    if not data:
        return None
    optimizer = str(data.get("optimizer") or "")
    score = re.search(r"\bcommon_score=([-+0-9.eE]+)", optimizer)
    source = re.search(r"\bsource=([^ ]+)", optimizer)
    planner_time = re.search(r"\btime=([-+0-9.eE]+)s", optimizer)
    selected = data.get("quality_report", {}).get("selected", {})
    cost_profile = selected.get("cost_profile")
    quality_profile = (cost_profile.get("name") if isinstance(cost_profile, dict)
                       else cost_profile)
    mode_names = {0: "A", 1: "O", 2: "S"}
    return {
        "plan_id": data.get("plan_id"),
        "search_success": str(data.get("search") or "").startswith("search_success"),
        "optimizer_status": optimizer.split(" ", 1)[0] if optimizer else None,
        "optimizer_source": source.group(1) if source else None,
        "common_score": float(score.group(1)) if score else None,
        "minlp_candidate_common_score": (
            audits[-1].get("candidate_common_score") if audits else None),
        "seed_common_score": audits[-1].get("seed_common_score") if audits else None,
        "model_objective": audits[-1].get("model_objective") if audits else None,
        "model_objective_components": (
            audits[-1].get("model_objective_components") if audits else None),
        "planner_seconds": float(planner_time.group(1)) if planner_time else None,
        "timed_sweep_clear": data.get("timed_sweep_clear"),
        "sweep_samples": data.get("sweep_samples"),
        "terminal_ok": data.get("terminal_ok"),
        "position_error_m": data.get("position_error"),
        "heading_error_rad": data.get("heading_error"),
        "mode_runs": [{**run, "name": mode_names.get(run["mode"], "?")}
                       for run in data.get("mode_runs", [])],
        "reverse_velocity_detected": data.get("reverse_velocity_detected"),
        "quality_score": selected.get("score"),
        "quality_profile": quality_profile,
        "reverse_body_projection_detected": data.get("reverse_body_projection_detected"),
        "independent_dynamics": data.get("independent_dynamics"),
        "independent_direction": data.get("independent_direction"),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", required=True, help="case key in the YAML manifest")
    parser.add_argument("--cases-file", type=Path, default=MANIFEST)
    parser.add_argument("--config-file", type=Path, default=PARAMS,
                        help="base ROS parameters; default is the current demo config")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="isolated ROS master port; protected ports are rejected")
    parser.add_argument("--joint", choices=("on", "off"), default="off")
    parser.add_argument("--cbc", choices=("on", "off"), default="off")
    parser.add_argument("--capture-seed", action="store_true")
    parser.add_argument("--expect-geometry-only", action="store_true",
                        help="verify a negative execution case; never counts as a feasible trajectory")
    parser.add_argument("--replay-seed", type=Path)
    parser.add_argument("--set", dest="overrides", action="append", default=[],
                        metavar="PATH=VALUE", help="override an existing YAML key")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="planner and checker deadline in seconds")
    parser.add_argument("--output-dir", type=Path,
                        help="run directory; default is a timestamped relust/probes path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    suffix = f"{args.case}_{args.joint}_cbc{args.cbc}_{run_id}"
    run_dir = args.output_dir or (ROOT / "relust/probes" / suffix)
    if not run_dir.is_absolute():
        run_dir = ROOT / run_dir
    run_dir = run_dir.resolve()
    if run_dir.exists() and any(run_dir.iterdir()):
        print(f"ERROR: output directory is not empty: {run_dir}", file=sys.stderr)
        return 2
    run_dir.mkdir(parents=True, exist_ok=True)

    result = {
        "schema": "expv3_probe_run_v1",
        "run_id": run_id,
        "started_at_utc": utc_now(),
        "case": args.case,
        "joint": args.joint,
        "cbc": args.cbc,
        "expected_outcome": "GEOMETRY_ONLY" if args.expect_geometry_only else "FEASIBLE",
        "master_port": args.port,
        "run_dir": str(run_dir),
        "status": "running",
        "artifacts": {},
        "cleanup": {"ros_master_stopped": False, "port_released": False},
    }
    result_path = run_dir / "run.json"
    log_streams = []
    roscore = roslaunch = probe = None
    preexisting_ros_processes: dict[int, str] = {}
    success = False
    prefix = None
    try:
        if args.timeout <= 0 or not math.isfinite(args.timeout):
            raise ValueError("--timeout must be a finite positive number")
        assert_port_available(args.port)
        if args.cbc == "on" and args.joint != "on":
            raise ValueError("--cbc on requires --joint on")
        if args.capture_seed and args.replay_seed:
            raise ValueError("--capture-seed and --replay-seed are mutually exclusive")

        with args.cases_file.open("r", encoding="utf-8") as stream:
            manifest = yaml.safe_load(stream)
        if manifest.get("schema") != "expv3_eval_cases_v1":
            raise ValueError("unsupported case manifest schema")
        cases = manifest.get("cases", {})
        if args.case not in cases:
            raise KeyError(f"unknown case {args.case}; available: {', '.join(cases)}")
        case = dict(cases[args.case])
        validate_case(args.case, case)
        start = finite_pose(case["start"], "start")
        goal = finite_pose(case["goal"], "goal")
        source_map = (ROOT / case["map_dir"]).resolve()
        effective_map = source_map
        if float(case["map_scale"]) != 1.0:
            effective_map = run_dir / "runtime_map"
            scale_map(source_map, effective_map, float(case["map_scale"]))
        map_digest, map_files = tree_hash(effective_map)

        base_config_path = args.config_file.resolve()
        with base_config_path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        for key, value in parse_overrides(args.overrides):
            set_existing(config, key, value)
        set_existing(config, "joint.enabled", args.joint == "on")
        set_existing(config, "joint.use_cbc_seed_incumbent", args.cbc == "on")
        set_existing(config, "joint.seed_trace_file", "")
        set_existing(config, "joint.seed_replay_file", "")
        # Keep every backend's complete final geometry beside its probe.
        # Legacy native playback is still audited as a separate time model.
        config.setdefault("quality", {})["report_dir"] = str(run_dir / "reports")
        seed_trace_path = run_dir / "joint_seed.json"
        if args.capture_seed:
            set_existing(config, "joint.seed_trace_file", str(seed_trace_path))
        replay_record = None
        replay_path = None
        map_signature = map_grid_signature(effective_map, config)
        if args.replay_seed:
            original_replay_path = args.replay_seed.expanduser().resolve()
            if not original_replay_path.is_file():
                raise FileNotFoundError(
                    f"frozen seed does not exist: {original_replay_path}")
            replay_path = run_dir / "replay_seed_input.json"
            if original_replay_path != replay_path.resolve():
                shutil.copy2(original_replay_path, replay_path)
            replay_record = validate_replay_seed(replay_path, map_signature,
                                                 start, goal)
            set_existing(config, "joint.seed_replay_file", str(replay_path))

        plugin = PLUGIN_DIR / "libcasadi_nlpsol_bonminseed.so"
        if args.cbc == "on" and not plugin.is_file():
            raise FileNotFoundError(
                f"CBC seed plugin is not built: {plugin}; build tools/build_bonminseed_plugin.sh first")
        config_path = run_dir / "params.yaml"
        with config_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(config, stream, sort_keys=False, allow_unicode=True)

        case_record = {
            "case": args.case,
            **case,
            "start": start,
            "goal": goal,
            "source_map_dir": str(source_map),
            "effective_map_dir": str(effective_map),
            "map_tree_sha256": map_digest,
            "map_files": map_files,
            "optimizer_map_signature": map_signature,
        }
        atomic_json(run_dir / "case.json", case_record)
        code_paths = [
            ROOT / "catkin_ws/src/fwis_v3_planner/src/planner_node.cpp",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/quality.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/retiming.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_geometry.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_retiming.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_optimizer.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/segmented_smoothing.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/mode_master.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/geometry_incumbent.py",
            ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/planning_display.py",
            ROOT / "catkin_ws/src/fwis_v3_playback/scripts/playback.py",
            ROOT / "catkin_ws/src/fwis_v3_demo/launch/demo.launch",
            ROOT / "tests/ros_map_probe.py",
            ROOT / "tools/trajectory_direction_audit.py",
            ROOT / "tests/ros_parking_goals_smoke.py",
            ROOT / "tools/scale_map_runtime.py",
            base_config_path, Path(__file__).resolve(), args.cases_file.resolve(),
        ]
        code_state = {
            str(path.relative_to(ROOT) if ROOT in path.parents else path): {
                "sha256": sha256_file(path), "mtime_ns": path.stat().st_mtime_ns,
            }
            for path in code_paths
        }
        result.update({
            "finished_input_validation_at_utc": utc_now(),
            "case_data": case_record,
            "requested_config": config,
            "config_file": str(config_path),
            "base_config_file": str(base_config_path),
            "code_state": code_state,
            "replay_seed": str(replay_path) if replay_path else None,
            "replay_seed_score": replay_record.get("score") if replay_record else None,
            "seed_trace_file": str(seed_trace_path) if args.capture_seed else None,
            "ros_master_uri": f"http://127.0.0.1:{args.port}",
            "checker": {"sampling_step_m": 0.005,
                        "xy_tolerance_m": float(case["xy_tol"]),
                        "yaw_tolerance_rad": float(case["yaw_tol"])},
        })

        prefix = ros_shell_prefix(args.port)
        master_uri = f"http://127.0.0.1:{args.port}"
        preexisting_ros_processes = project_ros_processes(master_uri)
        result["preexisting_project_ros_processes"] = preexisting_ros_processes
        roscore, log = start_logged(["roscore", "-p", str(args.port)],
                                    run_dir / "roscore.log", prefix)
        log_streams.append(log)
        wait_for_master(args.port, roscore)
        launch_args = [
            "roslaunch", "fwis_v3_demo", "demo.launch",
            f"scene:={case['scene']}", f"map_dir:={effective_map}",
            "rviz:=false", "auto_goal:=false", f"config_file:={config_path}",
        ]
        roslaunch, log = start_logged(launch_args, run_dir / "roslaunch.log", prefix)
        log_streams.append(log)
        time.sleep(2.0)
        if roslaunch.poll() is not None:
            raise RuntimeError(f"roslaunch exited early with code {roslaunch.returncode}")

        probe_args = [
            "python3", str(ROOT / "tests/ros_map_probe.py"),
            "--map-name", args.case,
            "--start", *[str(value) for value in start],
            "--goal", *[str(value) for value in goal],
            "--xy-tol", str(case["xy_tol"]),
            "--yaw-tol", str(case["yaw_tol"]),
            "--timeout", str(args.timeout),
            "--output", str(run_dir / "probe_result.json"),
        ]
        if args.expect_geometry_only:
            probe_args.append("--expect-geometry-only")
        probe, log = start_logged(probe_args, run_dir / "probe-client.log", prefix)
        log_streams.append(log)
        try:
            probe.wait(timeout=args.timeout + 40.0)
        except subprocess.TimeoutExpired as exc:
            terminate_process(probe)
            raise TimeoutError("probe client exceeded its total deadline") from exc
        result["probe_exit_code"] = probe.returncode
        probe_result_path = run_dir / "probe_result.json"
        if probe_result_path.is_file():
            with probe_result_path.open("r", encoding="utf-8") as stream:
                result["probe_result"] = json.load(stream)
        if probe.returncode != 0:
            raise RuntimeError(f"probe client returned {probe.returncode}")
        probe_data = result.get("probe_result", {})
        if args.expect_geometry_only:
            if probe_data.get("geometry_only_check", {}).get("pass_check") is not True:
                raise RuntimeError("geometry-only negative-case checker did not PASS")
        elif (probe_data.get("timed_sweep_clear") is not True or
                probe_data.get("terminal_ok") is not True or
                probe_data.get("strictly_increasing_time") is not True or
                probe_data.get("independent_direction", {}).get("pass_check") is not True):
            raise RuntimeError("independent checker did not report a complete PASS")
        if args.capture_seed and not seed_trace_path.is_file():
            raise RuntimeError("requested joint seed trace was not written")
        success = True
        result["status"] = "pass"
    except Exception as exc:
        result["status"] = "fail"
        result["error"] = f"{type(exc).__name__}: {exc}"
        print(f"ERROR: {result['error']}", file=sys.stderr)
    finally:
        terminate_process(probe)
        remaining_nodes = []
        current_ros_processes = project_ros_processes(
            f"http://127.0.0.1:{args.port}")
        started_nodes = {
            pid: command for pid, command in current_ros_processes.items()
            if pid not in preexisting_ros_processes
        }
        result["cleanup"]["launched_node_processes"] = started_nodes
        if started_nodes and prefix is not None:
            remaining_nodes = stop_launch_children(
                prefix, started_nodes, run_dir / "rosnode-shutdown.log")
        result["cleanup"]["launched_node_pids"] = sorted(started_nodes)
        result["cleanup"]["launched_node_processes_remaining"] = remaining_nodes
        terminate_process(roslaunch)
        result["cleanup"]["roslaunch_stopped"] = (
            roslaunch is None or roslaunch.poll() is not None)
        terminate_process(roscore)
        result["cleanup"]["ros_master_stopped"] = (
            roscore is None or roscore.poll() is not None)
        result["cleanup"]["port_released"] = not port_is_listening(args.port)
        result["cleanup"]["all_started_processes_stopped"] = all(
            process is None or process.poll() is not None
            for process in (probe, roslaunch, roscore)) and not remaining_nodes
        result["finished_at_utc"] = utc_now()
        try:
            result["optimizer_audits"] = parse_optimizer_audits(
                run_dir / "roslaunch.log")
        except Exception as exc:
            result["optimizer_audits"] = []
            result["optimizer_audit_parse_error"] = (
                f"{type(exc).__name__}: {exc}")
        result["artifacts"].update({
            "run_json": str(result_path),
            "case_json": str(run_dir / "case.json"),
            "params_yaml": str(run_dir / "params.yaml"),
            "roscore_log": str(run_dir / "roscore.log"),
            "roslaunch_log": str(run_dir / "roslaunch.log"),
            "probe_client_log": str(run_dir / "probe-client.log"),
            "rosnode_shutdown_log": str(run_dir / "rosnode-shutdown.log"),
            "probe_result_json": str(run_dir / "probe_result.json"),
            "joint_seed_json": str(run_dir / "joint_seed.json")
            if args.capture_seed else None,
            "replay_seed_json": str(run_dir / "replay_seed_input.json")
            if args.replay_seed else None,
        })
        if not result["cleanup"]["port_released"]:
            result["status"] = "fail"
            result["error"] = "isolated ROS master port remained occupied after cleanup"
            success = False
        if not result["cleanup"]["all_started_processes_stopped"]:
            result["status"] = "fail"
            result["error"] = "one or more started ROS processes did not stop"
            success = False
        for stream in log_streams:
            stream.close()
        try:
            atomic_json(result_path, result)
        except Exception as exc:
            print(f"ERROR writing run metadata: {exc}", file=sys.stderr)
            success = False

    print(json.dumps({
        "status": result["status"],
        "case": args.case,
        "run_dir": str(run_dir),
        "probe_summary": summarize_probe(
            result.get("probe_result"), result.get("optimizer_audits")),
        "cleanup": result["cleanup"],
        "error": result.get("error"),
    }, ensure_ascii=False, indent=2))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
