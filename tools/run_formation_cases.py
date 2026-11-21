#!/usr/bin/env python3
"""List and replay formation scenarios captured through run_demo.sh."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
from queue import Empty, Queue
import signal
import socket
import subprocess
import sys
import tempfile
import time
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "catkin_ws/src/fwis_v3_demo"
SUITE = ROOT / "work_flow/test_scenarios"
SCHEMA = "formation_scenario/v1"
BACKENDS = ("uniform_v02", "virtual_rigid_a")
PLANNER_ARGS = (
    "max_expansions", "uniform_max_seconds", "uniform_translation_step",
    "uniform_direct_goal_translation", "uniform_global_translation_route",
    "uniform_heading_aligned_route", "uniform_heading_transition_route",
    "uniform_continuous_a", "uniform_formation_raster", "uniform_preview_checks",
    "uniform_collision_margin",
    "uniform_sweep_spacing", "uniform_publication_dt", "uniform_goal_xy_tol",
    "uniform_goal_formation_heading_free",
    "uniform_goal_psi_tol", "playback_start_timeout_s", "playback_finish_grace_s",
)
VIRTUAL_ARGS = (
    "virtual_a_guidance_mode", "virtual_a_corridor_enforcement",
    "virtual_a_formation_corridor_scope", "virtual_a_goal_policy",
    "virtual_a_planning_budget_s", "virtual_a_planning_timeout_s",
)
PLAN_ID = 600000


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    file = path.with_suffix(".tmp")
    file.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")
    file.replace(path)


def make_profile(path, map_dir, scale, launch_args):
    import yaml

    args = dict(arg.split(":=", 1) for arg in launch_args)
    backend = args["backend"]
    launch = ElementTree.parse(DEMO / "launch/formation_demo.launch")
    defaults = {element.attrib["name"]: element.attrib["default"]
                for element in launch.findall("arg")}
    keys = PLANNER_ARGS + (VIRTUAL_ARGS if backend == "virtual_rigid_a" else ())
    source = Path(map_dir).resolve()
    transport = Path(args.get("transport_config", DEMO / "config/transport_formation.yaml"))
    write_json(Path(path), {
        "schema": SCHEMA,
        "backend": backend,
        "map": {"directory": str(source.relative_to(ROOT) if ROOT in source.parents else source),
                "scale": float(scale)},
        "planner": {key: args.get(key, defaults[key]) for key in keys},
        "transport": yaml.safe_load(transport.read_text(encoding="utf-8")),
    })


def load_case(path):
    case = read_json(path)
    if case["schema"] != SCHEMA or case["case_id"] != Path(path).stem:
        raise ValueError("case schema or case_id does not match the file")
    if case["backend"] not in BACKENDS:
        raise ValueError("unsupported backend")
    keys = PLANNER_ARGS + (VIRTUAL_ARGS if case["backend"] == "virtual_rigid_a" else ())
    planner = case["planner"]
    if (not isinstance(planner, dict) or set(planner) - set(keys) or
            not {"uniform_goal_xy_tol", "uniform_goal_psi_tol"} <= planner.keys()):
        raise ValueError("planner parameters do not match the case schema")
    scale = case["map"]["scale"]
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("map scale must be positive and finite")
    signature = case["map"]["grid_sha256"]
    if len(signature) != 64 or any(char not in "0123456789abcdef" for char in signature):
        raise ValueError("missing or invalid map fingerprint")
    expectation = case["expect"]
    if expectation["outcome"] not in ("success", "rejected"):
        raise ValueError("expect.outcome must be success or rejected")
    if expectation["outcome"] == "rejected" and (
            not isinstance(expectation.get("reason"), str) or not expectation["reason"].strip()):
        raise ValueError("rejected cases require expect.reason")
    if case["request"]["continuation_plan_id"] or case["request"]["continuation_model_sha256"]:
        raise ValueError("scenario requires an independent start")
    return case


class ReplayFailure(RuntimeError):
    def __init__(self, layer, code, detail):
        super().__init__(detail)
        self.layer = layer
        self.code = code


def run_ros(case, map_dir, transport, state, timeout, log_dir, log_path):
    with socket.socket() as available:
        available.bind(("127.0.0.1", 0))
        port = available.getsockname()[1]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:{}".format(port)
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    os.environ.pop("ROS_IP", None)
    os.environ["ROS_HOME"] = str(log_dir)
    os.environ["ROS_LOG_DIR"] = str(log_dir / "ros_logs")

    command = [
        "roslaunch", "fwis_v3_demo", "formation_demo.launch",
        "map_dir:={}".format(map_dir), "transport_config:={}".format(transport),
        "backend:={}".format(case["backend"]), "rviz:=false", "auto_goal:=false",
        "goal_bridge:=false",
        *("{}:={}".format(name, value) for name, value in case["planner"].items()),
    ]
    with log_path.open("w", encoding="utf-8") as log:
        launch = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        try:
            check_ros(case, state, timeout, launch)
        finally:
            if launch.poll() is None:
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()


def check_ros(case, state, timeout, launch):
    import rosgraph
    import rospy
    from fwis_v3_msgs.msg import FormationPlanRequest, FormationPlaybackPlan
    from genpy.message import fill_message_args
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String

    events = Queue()
    startup_deadline = time.monotonic() + 35

    def ensure_active(layer, deadline):
        if launch.poll() is not None:
            raise ReplayFailure(layer, "ROS_LAUNCH_EXITED", "see launch.log")
        if time.monotonic() >= deadline:
            raise ReplayFailure(layer, "REPLAY_TIMEOUT", "timed out waiting for " + layer)

    while not rosgraph.is_master_online():
        ensure_active("startup", startup_deadline)
        time.sleep(0.1)
    rospy.init_node("formation_case_runner", disable_signals=True)
    backend = case["backend"]
    request = FormationPlanRequest()
    fill_message_args(request, [case["request"]])
    request.plan_id = PLAN_ID

    def envelope_fields(message):
        if message.plan_id != PLAN_ID:
            return None
        trajectory = message.trajectory
        goal = trajectory.formation_poses[-1] if trajectory.formation_poses else None
        return {
            "plan_id": message.plan_id, "backend": message.backend,
            "formation_id": message.formation_id,
            "model_sha256": message.model_sha256, "certification": message.certification,
            "contains_model": bool(message.model_json),
            "sample_count": len(trajectory.times_from_start),
            "goal": None if goal is None else [goal.x, goal.y, goal.theta],
        }

    subscribers = [
        rospy.Subscriber(topic, String, lambda message, key=key:
                         events.put((key, json.loads(message.data))), queue_size=20)
        for key, topic in (
            ("job", "/formation/job_state"), ("optimizer", "/formation/optimizer_status"),
            ("quality", "/formation/quality_report"), ("playback", "/formation/playback_status"))
    ]
    subscribers.append(rospy.Subscriber(
        "/formation/playback_plan", FormationPlaybackPlan,
        lambda message: events.put(("envelope", envelope_fields(message))), queue_size=5))
    publisher = rospy.Publisher("/formation/plan_request", FormationPlanRequest, queue_size=1, latch=True)

    def wait_for(predicate, layer, deadline):
        while not predicate():
            ensure_active(layer, deadline)
            try:
                key, data = events.get(timeout=0.1)
            except Empty:
                continue
            if not isinstance(data, dict):
                continue
            if key == "job" or data.get("plan_id") == PLAN_ID:
                state[key] = data
            if key == "envelope" and data["contains_model"]:
                state["executable_envelope_seen"] = True
            if key == "playback" and data.get("state") == "CLEARED":
                state["clear_count"] = state.get("clear_count", 0) + 1
                state["cleared"] = data

    try:
        wait_for(lambda: state.get("job", {}).get("map_ready") and
                 publisher.get_num_connections() >= 2 and
                 all(sub.get_num_connections() for sub in subscribers),
                 "startup", startup_deadline)
        actual = state["job"].get("map_signature")
        if actual != case["map"]["grid_sha256"]:
            raise ReplayFailure("input", "MAP_FINGERPRINT_MISMATCH",
                                "expected {}, got {}".format(case["map"]["grid_sha256"], actual))

        if backend == "virtual_rigid_a":
            reset = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
            wait_for(lambda: reset.get_num_connections() > 0, "initial_reset", startup_deadline)
            initial = PoseWithCovarianceStamped()
            initial.header = request.header
            qf = request.formation_start
            initial.pose.pose.position.x = qf.x
            initial.pose.pose.position.y = qf.y
            initial.pose.pose.orientation.z = math.sin(qf.theta / 2)
            initial.pose.pose.orientation.w = math.cos(qf.theta / 2)
            reset.publish(initial)
            wait_for(lambda: state.get("job", {}).get("initial_reset_ready") is True,
                     "initial_reset", startup_deadline)

        clear_count = state.get("clear_count", 0)
        deadline = time.monotonic() + timeout
        request.header.stamp = rospy.Time.now()
        sent = time.monotonic()
        publisher.publish(request)
        wait_for(lambda: state.get("job", {}).get("plan_id") == PLAN_ID and
                 state["job"]["state"] in ("HOLD_END", "FAILED"), "job_terminal", deadline)
        if state["job"]["state"] == "HOLD_END":
            wait_for(lambda: state.get("envelope", {}).get("contains_model") and
                     state.get("quality", {}).get("executable") and
                     state.get("playback", {}).get("state") == "HOLD_END" and
                     state["playback"].get("executable") and
                     state.get("optimizer", {}).get("node_state") == "HOLD_END",
                     "terminal_evidence", deadline)
        else:
            wait_for(lambda: state.get("quality", {}).get("executable") is False and
                     state.get("optimizer", {}).get("executable") is False and
                     state.get("envelope", {}).get("contains_model") is False and
                     state.get("clear_count", 0) > clear_count,
                     "failure_cleanup", deadline)
        state["elapsed_s"] = time.monotonic() - sent
        assess(case, state)
    finally:
        rospy.signal_shutdown("case replay finished")


def assess(case, state):
    expected = case["expect"]
    quality = state["quality"]
    if state["job"]["state"] == "FAILED":
        envelope = state["envelope"]
        if (state.get("executable_envelope_seen") or quality.get("trajectory_published")
                or envelope["model_sha256"] or envelope["sample_count"]):
            raise ReplayFailure("publication", "EXECUTABLE_BEFORE_FAILURE",
                                "an executable envelope was published before the job failed")
        if expected["outcome"] == "rejected" and quality["reason"] == expected["reason"]:
            return
        raise ReplayFailure(quality.get("first_failure_layer") or "planning",
                            quality["reason"], quality.get("detail") or quality["reason"])

    if expected["outcome"] == "rejected":
        raise ReplayFailure("expectation", "UNEXPECTED_SUCCESS",
                            "expected rejection " + expected["reason"])

    envelope = state["envelope"]
    playback = state["playback"]
    digest = envelope["model_sha256"]
    if (not digest or envelope["formation_id"] != case["request"]["formation_id"]
            or envelope["backend"] != case["backend"]
            or any(state[key].get("model_sha256") != digest
                   for key in ("quality", "job", "playback", "optimizer"))):
        raise ReplayFailure("publication", "MODEL_IDENTITY_MISMATCH", "terminal model digests differ")
    if quality.get("request_map_signature") != case["map"]["grid_sha256"]:
        raise ReplayFailure("input", "REQUEST_MAP_MISMATCH", "the admitted request used another map")
    if case["backend"] == "virtual_rigid_a":
        audits = quality.get("virtual_result", {}).get("audits", {})
        audited = all((audits.get(key) or {}).get("passed") is True
                      for key in ("geometry", "dynamics", "collision"))
    else:
        audit = quality.get("evaluation") or {}
        audited = all(audit.get(key) is True for key in
                      ("independent_audit", "feasible", "collision_certificate"))
    if not audited:
        raise ReplayFailure("audit", "AUDIT_INCOMPLETE", "no successful independent audit")
    vehicles = playback["vehicles"]
    if (not playback.get("terminal")
            or {vehicle["vehicle_id"] for vehicle in vehicles}
            != {vehicle["vehicle_id"] for vehicle in case["request"]["vehicles"]}
            or any(abs(speed) > 1e-7 for vehicle in vehicles for speed in (
                vehicle["center_speed_mps"], vehicle.get("yaw_rate_radps", 0.0),
                *vehicle.get("wheel_rolling_speeds_mps", ())))):
        raise ReplayFailure("playback", "TERMINAL_HOLD_INVALID", "vehicles must hold at rest")

    goal = case["request"]["formation_goal"]
    x, y, yaw = envelope["goal"]
    xy_error = math.hypot(x - goal["x"], y - goal["y"])
    yaw_error = abs(math.remainder(yaw - goal["theta"], math.tau))
    state["goal_error"] = {"position_m": xy_error, "yaw_rad": yaw_error}
    defaults = ((0.005, math.radians(0.2)) if case["backend"] == "virtual_rigid_a"
                else (float(case["planner"]["uniform_goal_xy_tol"]),
                      float(case["planner"]["uniform_goal_psi_tol"])))
    free_yaw = (case["backend"] == "uniform_v02" and
                str(case["planner"].get("uniform_goal_formation_heading_free", "false")).lower() == "true")
    if xy_error > defaults[0] + 1e-9 or (not free_yaw and yaw_error > defaults[1] + 1e-12):
        raise ReplayFailure("terminal", "GOAL_TOLERANCE_EXCEEDED",
                            "{:.6g} m, {:.6g} rad".format(xy_error, yaw_error))


def replay(path, result_dir, timeout):
    state = {}
    started = time.monotonic()
    try:
        from scale_map_runtime import scale_map

        case = load_case(path)
        source = (ROOT / case["map"]["directory"]).resolve()
        with tempfile.TemporaryDirectory(prefix="4wis-scenario-") as temporary:
            runtime = Path(temporary)
            scaled_map = runtime / source.name
            scale_map(source, scaled_map, case["map"]["scale"])
            config = runtime / "transport.json"
            write_json(config, case["transport"])
            run_ros(case, scaled_map, config, state, timeout, runtime / "ros",
                    result_dir / "launch.log")
        result = {"passed": True, "failure": None}
    except ReplayFailure as exc:
        result = {"passed": False, "failure": {
            "layer": exc.layer, "code": exc.code, "detail": str(exc)}}
    except Exception as exc:
        result = {"passed": False, "failure": {
            "layer": "runner", "code": type(exc).__name__, "detail": str(exc)}}
    write_json(result_dir / "result.json", {
        **result, "worker_wall_s": time.monotonic() - started, "evidence": state})


def case_digest(case):
    fields = {key: case[key] for key in ("map", "backend", "transport", "planner", "request", "expect")}
    data = json.dumps(fields, sort_keys=True, allow_nan=False).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def batch(args, paths):
    overlay = ROOT / "catkin_ws/devel/setup.bash"
    if not overlay.is_file():
        raise ValueError("请先通过 run_demo.sh 构建 ROS overlay")
    baseline = read_json(args.baseline) if args.baseline else {}
    previous = {case["case_id"]: case for case in baseline.get("cases", [])}
    run_dir = Path(args.output_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-"),
                                    dir=run_dir)).resolve()
    report_path = run_dir / "report.json"
    report = {"label": args.label, "baseline": args.baseline, "complete": False,
              "jobs": args.jobs, "planner_overrides": dict(args.planner), "cases": []}
    print("报告 {}".format(report_path), flush=True)

    def update_report():
        report["failures"] = [result for result in report["cases"] if not result["passed"]]
        rows = report["cases"]
        def p95(key, passed_only=False):
            data = sorted(row[key] for row in rows
                          if row.get(key) is not None and (row["passed"] or not passed_only))
            return None if not data else data[max(0, math.ceil(0.95 * len(data)) - 1)]

        report["metrics"] = {
            "attempted": len(rows), "passed": sum(row["passed"] for row in rows),
            "search_timeouts": sum((row["failure"] or {}).get("code") == "TIME_BUDGET_EXHAUSTED"
                                   for row in rows),
            "planning_p95_s": p95("planner_s"),
            "successful_end_to_end_p95_s": p95("end_to_end_s", passed_only=True),
            "successful_trajectory_p95_s": p95("trajectory_s", passed_only=True),
        }
        write_json(report_path, report)

    def run_case(path):
        result_dir = run_dir / path.stem
        result_dir.mkdir()
        print("运行 {}".format(path.stem), flush=True)
        try:
            case = load_case(path)
            digest = case_digest(case)
            keys = PLANNER_ARGS + (VIRTUAL_ARGS if case["backend"] == "virtual_rigid_a" else ())
            if any(key not in keys for key, _ in args.planner):
                raise ValueError("planner override is unavailable for " + case["backend"])
            case["planner"].update(args.planner)
            effective_digest = case_digest(case)
        except (ValueError, OSError, KeyError, TypeError) as exc:
            case, digest, effective_digest = {}, None, None
            result = {"passed": False, "failure": {
                "layer": "input", "code": "INVALID_CASE", "detail": str(exc)}}
        else:
            inputs = result_dir / "input"
            inputs.mkdir()
            snapshot = inputs / path.name
            write_json(snapshot, case)
            env = os.environ.copy()
            env.pop("ANTHROPIC_AUTH_TOKEN", None)
            env["PYTHONUNBUFFERED"] = "1"
            with (result_dir / "worker.log").open("w", encoding="utf-8") as log:
                worker = subprocess.run([
                    "bash", "-c", 'source /opt/ros/noetic/setup.bash && source "$1" && shift && exec "$@"',
                    "formation-case", str(overlay), "/usr/bin/python3", str(Path(__file__).resolve()),
                    "_replay", str(snapshot), str(result_dir), str(args.timeout),
                ], stdout=log, stderr=subprocess.STDOUT, env=env, cwd=result_dir)
            artifact = result_dir / "result.json"
            if worker.returncode != 0 or not artifact.is_file():
                result = {"passed": False, "failure": {
                    "layer": "runner", "code": "WORKER_EXIT", "detail": "see worker.log"}}
            else:
                raw = read_json(artifact)
                evidence = raw["evidence"]
                result = {"passed": raw["passed"], "failure": raw["failure"],
                          "planner_s": evidence.get("quality", {}).get("wall_time_s"),
                          "end_to_end_s": evidence.get("elapsed_s"),
                          "trajectory_s": evidence.get("quality", {}).get("T_end")}
        result.update(case_id=path.stem, name=case.get("name", path.stem),
                      input_sha256=digest, effective_input_sha256=effective_digest,
                      artifacts=result_dir.name)
        old = previous.get(path.stem)
        if old:
            if old["input_sha256"] != digest:
                result["baseline_change"] = "input_changed"
            elif old["passed"] != result["passed"]:
                result["baseline_change"] = "fixed" if result["passed"] else "regressed"
            elif not result["passed"] and any(
                    old["failure"][key] != result["failure"][key] for key in ("layer", "code")):
                result["baseline_change"] = "failure_changed"
            else:
                result["baseline_change"] = "unchanged"
        return result

    update_report()
    with ThreadPoolExecutor(max_workers=args.jobs) as executor:
        futures = [executor.submit(run_case, path) for path in paths]
        for future in as_completed(futures):
            result = future.result()
            report["cases"].append(result)
            report["cases"].sort(key=lambda row: row["case_id"])
            if result["passed"]:
                print("通过 {}".format(result["case_id"]), flush=True)
            else:
                print("失败 {} {} {}".format(result["case_id"], result["failure"]["layer"],
                                            result["failure"]["code"]), flush=True)
            update_report()

    report["complete"] = True
    update_report()
    print("通过 {}/{}，报告 {}".format(len(paths) - len(report["failures"]), len(paths),
                                    report_path))
    return int(bool(report["failures"]))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "_profile":
        make_profile(*sys.argv[2:5], sys.argv[5:])
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == "_replay":
        replay(Path(sys.argv[2]), Path(sys.argv[3]), float(sys.argv[4]))
        return 0

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("list", "run"))
    parser.add_argument("--cases-dir", type=Path, default=SUITE / "cases")
    parser.add_argument("--case", action="append", help="case ID, repeatable")
    parser.add_argument("--label", default="")
    parser.add_argument("--baseline", help="previous report.json")
    parser.add_argument("--output-dir", type=Path, default=SUITE / "runs")
    parser.add_argument("--timeout", type=float, default=360, help="seconds per replay after ROS startup")
    parser.add_argument("--jobs", type=int, default=1, help="number of isolated ROS replays in parallel")
    parser.add_argument("--planner", action="append", default=[], metavar="NAME:=VALUE",
                        help="override a planner launch value for a comparison")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a positive finite number")
    if args.jobs <= 0:
        parser.error("--jobs must be positive")
    overrides = []
    for value in args.planner:
        key, separator, setting = value.partition(":=")
        if not separator or key not in PLANNER_ARGS + VIRTUAL_ARGS or not setting:
            parser.error("unknown planner override " + value)
        overrides.append((key, setting))
    args.planner = overrides
    paths = sorted(Path(args.cases_dir).resolve().glob("*.json"))
    if args.case:
        missing = set(args.case) - {path.stem for path in paths}
        if missing:
            parser.error("unknown case IDs: " + ", ".join(sorted(missing)))
        paths = [path for path in paths if path.stem in args.case]
    if not paths:
        parser.error("no scenarios under {}".format(args.cases_dir))
    if args.command == "list":
        invalid = False
        for path in paths:
            try:
                case = load_case(path)
                print("{}  {}  {}  {}".format(
                    path.stem, case.get("name", path.stem), case["backend"], case["expect"]["outcome"]))
            except (ValueError, OSError, KeyError, TypeError) as exc:
                print("{}  INVALID_CASE  {}".format(path.stem, exc))
                invalid = True
        return int(invalid)
    return batch(args, paths)


if __name__ == "__main__":
    raise SystemExit(main())
