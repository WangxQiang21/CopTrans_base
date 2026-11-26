#!/usr/bin/env python3
"""Exercise the ordinary public defaults and the same two ROS inputs used by RViz."""
import argparse
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time
import yaml

from cases import LOCAL, ROOT
sys.path.insert(0, str(ROOT/"scripts"))
from source_fingerprint import RUNTIME_ARTIFACTS, receipt_fingerprint


ENTRY_FLAGS = {
    "single": [],
    "multi": ["--task", "multi", "--experiment", "full_route"],
    "interactive": ["--task", "multi", "--interactive", "--experiment", "full_route"],
}


def observed_install(entry, archive):
    env_file = archive / "environment.txt"
    command_file = archive / "command.txt"
    env = dict(line.split("=", 1) for line in env_file.read_text().splitlines() if "=" in line)
    argv = shlex.split(command_file.read_text())
    accepted_flags = [ENTRY_FLAGS[entry]]
    if entry == "multi":
        # Older automatic runs used this command before the interactive default.
        # collect() also verifies the resolved run was automatic.
        accepted_flags.append(["--task", "multi"])
    artifacts = json.loads(env.get("coptrans_installed_files", "{}"))
    digests = {f"{pkg}/{name}" for pkg, names in RUNTIME_ARTIFACTS.items() for name in names}
    valid = bool(argv and Path(argv[0]).parts[-2:] == ("scripts","test.sh") and argv[1:] in accepted_flags
                 and env.get("coptrans_build_type") == "Release" and set(artifacts) == digests
                 and all(re.fullmatch("[0-9a-f]{64}", v) for v in artifacts.values())
                 and receipt_fingerprint(artifacts) == env.get("coptrans_install_sha256")
                 and re.fullmatch("[0-9a-f]{64}", env.get("coptrans_source_sha256",""))
                 and re.fullmatch("[0-9a-f]{40}", env.get("git_commit","")))
    return dict(valid=valid, source_commit=env.get("git_commit"), source_sha256=env.get("coptrans_source_sha256"),
                install_sha256=env.get("coptrans_install_sha256"), installed_files=artifacts,
                argv=["scripts/test.sh", *argv[1:]] if argv else [],
                environment_sha256=sha256(env_file.read_bytes()).hexdigest(),
                command_sha256=sha256(command_file.read_bytes()).hexdigest())


def matches_qualified_runtime(receipts, runtime, relocation=None):
    from default_relocation import covers
    keys = {f"{pkg}/{name}" for pkg, names in RUNTIME_ARTIFACTS.items() if pkg != "fwis_bringup" for name in names}
    expected = {}
    for key in keys:
        matched = [value for filename, value in runtime.items() if Path(filename).as_posix().endswith("/" + key)]
        if len(matched) != 1:
            return False
        expected[key] = matched[0]
    if len(receipts)!=3 or {r.get("entry") for r in receipts}!={"single","multi","interactive"}:
        return False
    for receipt in receipts:
        install=receipt.get("install",{})
        if not receipt.get("native") or not install.get("valid"):
            return False
        observed=install.get("installed_files",{})
        if not all(observed.get(k)==v for k,v in expected.items()) and not covers(relocation or {},observed,expected):
            return False
    return True


def operate(run_id):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    fixture=yaml.safe_load(subprocess.check_output(
        [ROOT/"scripts/test.sh","--task","multi","--experiment","full_route","--dry-run"],text=True))
    active = yaml.safe_load((ROOT / "reports" / "runs" / run_id / "resolved_config.yaml").read_text())
    if active.get("assembly"):
        cases = yaml.safe_load((ROOT / "ros2_ws/src/fwis_bringup/config/cts_long_routes.yaml").read_text())
        fixture = {**fixture, **{key: (x, y, math.radians(heading)) for key, (x, y, heading)
                                in cases["cases"]["CTS_S2"].items()}}
    def set_pose(msg,endpoint):
        x,y,heading=fixture[endpoint]
        msg.position.x=float(x)
        msg.position.y=float(y)
        msg.orientation.z=math.sin(heading/2)
        msg.orientation.w=math.cos(heading/2)

    rclpy.init()
    node=rclpy.create_node("hybrid_v3_rviz_input_test")
    state={}
    def status(message):
        nonlocal state
        update=json.loads(message.data)
        if update.get("experiment_id")==run_id:
            state=update
    node.create_subscription(String,"/formation/interactive_status",status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sp=node.create_publisher(PoseWithCovarianceStamped,"/initialpose",5)
    gp=node.create_publisher(PoseStamped,"/move_base_simple/goal",5)
    started=goal=False
    end=time.monotonic()+fixture["timeout_s"]
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.15)
        phase=state.get("phase")
        if phase=="waiting_start" and not started and sp.get_subscription_count():
            msg=PoseWithCovarianceStamped()
            msg.header.frame_id="map"
            set_pose(msg.pose.pose,"start")
            sp.publish(msg)
            started=True
        elif phase=="waiting_goal" and started and not goal and gp.get_subscription_count():
            msg=PoseStamped()
            msg.header.frame_id="map"
            set_pose(msg.pose,"goal")
            gp.publish(msg)
            goal=True
        elif phase in ("finished","failed"):
            break
    node.destroy_node()
    rclpy.shutdown()
    return 0 if phase=="finished" and started and goal else 1


def collect(entry,archive):
    result_file=archive/"result.yaml"
    result=yaml.safe_load(result_file.read_text())
    config=yaml.safe_load((archive/"resolved_config.yaml").read_text())
    experiment=json.loads((archive/"experiment.json").read_text())
    duration=result.get("plan_duration_s")
    count=1 if entry=="single" else 3
    keys=set(tuple(v) for v in (result.get("routed_plan_keys") or {}).values())
    auth=any(e["event"]=="native_group_admission" and e.get("phase")=="AUTHORIZED" and
             e.get("model_sha256")==result.get("model_sha256") and (e["plan_id"],e["start_ns"]) in keys
             for e in map(json.loads,(archive/"timing_trace.jsonl").read_text().splitlines()))
    install=observed_install(entry,archive)
    native=(install["valid"] and result.get("exit_code")==0 and
            config["planner_backend"]==("single" if count==1 else "formation")+"_hybrid_joint_cpp" and
            result.get("planner_source")=="hybrid_joint" and config["tracking_implementation"]=="cpp" and auth and
            duration is not None and duration>0 and len(keys)==1 and result.get("complete")==list(range(1,count+1)) and
            bool(config.get("interactive"))==(entry=="interactive") and
            (entry!="interactive" or observed_stop(result)))
    return dict(entry=entry,run_id=archive.name,status=result["status"],reason=result["reason"],native=bool(native),
                planner_backend=config["planner_backend"],model_sha256=result.get("model_sha256"),
                map_sha256=experiment["map_sha256"],result_sha256=sha256(result_file.read_bytes()).hexdigest(),
                install=install)


def observed_stop(result):
    final=(result.get("termination_observation") or {}).get("final") or {}
    return result.get("status")=="passed" and final.get("window_complete") is True


def test_entry(entry,destination):
    runs=ROOT/"reports/runs"
    runs.mkdir(parents=True,exist_ok=True)
    before=set(p.name for p in runs.iterdir() if p.is_dir())
    command = (["scripts/interactive.sh"] if entry == "interactive" else
               ["scripts/test.sh", *ENTRY_FLAGS[entry]])
    resolved=yaml.safe_load(subprocess.check_output([*command,"--dry-run"],text=True,cwd=ROOT))
    expected=("single" if entry=="single" else "formation")+"_hybrid_joint_cpp"
    if resolved["planner_backend"]!=expected:
        raise ValueError(f"{entry}: default resolves to {resolved['planner_backend']}")
    if resolved["interactive"] != (entry == "interactive"):
        raise ValueError(f"{entry}: unexpected interactive mode")
    log=destination/(entry+".log")
    with log.open("w") as stream:
        proc=subprocess.Popen(command,cwd=ROOT,start_new_session=True,stdout=stream,stderr=subprocess.STDOUT)
        operator=None
        try:
            # A first public launch runs colcon before simulation begins.
            # The ordinary mission and RViz input deadlines start after that build.
            end=time.monotonic()+1800
            launch_ready=False
            while proc.poll() is None and time.monotonic()<end:
                archives=[p for p in runs.iterdir() if p.is_dir() and p.name not in before]
                if len(archives)>1:
                    raise RuntimeError("Multiple experiments appeared during a public default test")
                if archives and not launch_ready:
                    env_file=archives[0]/"environment.txt"
                    if env_file.is_file():
                        env=dict(line.split("=",1) for line in env_file.read_text().splitlines() if "=" in line)
                        if not (env.get("ROS_DOMAIN_ID") and env.get("coptrans_install_sha256")):
                            time.sleep(.3)
                            continue
                        launch_ready=True
                        end=time.monotonic()+resolved["timeout_s"]+120
                        if entry=="interactive":
                            operator=subprocess.Popen([sys.executable,__file__,"--operator",archives[0].name],
                                env=dict(os.environ,ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"],ROS_LOCALHOST_ONLY="1"))
                if operator is not None and operator.poll() is not None:
                    if operator.returncode:
                        raise RuntimeError("The RViz input sequence did not finish successfully")
                    # ROS reports finished before the configured stop observation is checkpointed.
                    result=archives[0]/"result.yaml"
                    if result.is_file() and observed_stop(yaml.safe_load(result.read_text()) or {}):
                        break
                time.sleep(.3)
        finally:
            if operator is not None and operator.poll() is None:
                operator.terminate()
                operator.wait(timeout=5)
            if proc.poll() is None:
                os.kill(proc.pid,signal.SIGINT)
                proc.wait(timeout=25)
        archives=[p for p in runs.iterdir() if p.is_dir() and p.name not in before]
    if len(archives)!=1:
        raise RuntimeError("Public entry point did not produce exactly one archive; see "+str(log))
    return collect(entry,archives[0])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--operator")
    parser.add_argument("--output",type=Path,default=LOCAL/"release_c6"/"default_runs.json")
    args=parser.parse_args()
    if args.operator:
        return operate(args.operator)
    out=args.output.parent
    out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for entry in ("single","multi","interactive"):
        rows.append(test_entry(entry,out))
        args.output.write_text(json.dumps(rows,indent=2)+"\n")
        if rows[-1]["status"]!="passed" or not rows[-1]["native"]:
            return 1
    return 0


if __name__=="__main__":
    raise SystemExit(main())
