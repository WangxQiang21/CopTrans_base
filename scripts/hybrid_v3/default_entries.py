#!/usr/bin/env python3
"""Exercise the ordinary public defaults and the same two ROS inputs used by RViz."""
import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml

from cases import LOCAL, ROOT


def operate(run_id):
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
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
    end=time.monotonic()+125
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.15)
        phase=state.get("phase")
        if phase=="waiting_start" and not started and sp.get_subscription_count():
            msg=PoseWithCovarianceStamped()
            msg.header.frame_id="map"
            msg.pose.pose.position.x=4.
            msg.pose.pose.position.y=5.6
            msg.pose.pose.orientation.w=1.
            sp.publish(msg)
            started=True
        elif phase=="waiting_goal" and started and not goal and gp.get_subscription_count():
            msg=PoseStamped()
            msg.header.frame_id="map"
            msg.pose.position.x=7.8
            msg.pose.position.y=6.3
            msg.pose.orientation.w=1.
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
    native=(config["planner_backend"]==("single" if count==1 else "formation")+"_hybrid_joint_cpp" and
            result.get("planner_source")=="hybrid_joint" and config["tracking_implementation"]=="cpp" and auth and
            duration is not None and duration>0 and len(keys)==1 and result.get("complete")==list(range(1,count+1)) and
            bool(config.get("interactive"))==(entry=="interactive"))
    return dict(entry=entry,run_id=archive.name,status=result["status"],reason=result["reason"],native=bool(native),
                planner_backend=config["planner_backend"],model_sha256=result.get("model_sha256"),
                map_sha256=experiment["map_sha256"],result_sha256=sha256(result_file.read_bytes()).hexdigest())


def test_entry(entry,destination):
    runs=ROOT/"reports/runs"
    before=set(p.name for p in runs.iterdir() if p.is_dir())
    command={ "single":["scripts/test.sh"],"multi":["scripts/test.sh","--task","multi"],
              "interactive":["scripts/interactive.sh"]}[entry]
    resolved=yaml.safe_load(subprocess.check_output([*command,"--dry-run"],text=True,cwd=ROOT))
    expected=("single" if entry=="single" else "formation")+"_hybrid_joint_cpp"
    if resolved["planner_backend"]!=expected:
        raise ValueError(f"{entry}: default resolves to {resolved['planner_backend']}")
    log=destination/(entry+".log")
    with log.open("w") as stream:
        proc=subprocess.Popen(command,cwd=ROOT,start_new_session=True,stdout=stream,stderr=subprocess.STDOUT)
        operator=None
        try:
            end=time.monotonic()+600
            while proc.poll() is None and time.monotonic()<end:
                archives=[p for p in runs.iterdir() if p.is_dir() and p.name not in before]
                if len(archives)>1:
                    raise RuntimeError("Multiple experiments appeared during a public default test")
                if entry=="interactive" and archives and operator is None:
                    env_file=archives[0]/"environment.txt"
                    if env_file.is_file():
                        env=dict(line.split("=",1) for line in env_file.read_text().splitlines() if "=" in line)
                        operator=subprocess.Popen([sys.executable,__file__,"--operator",archives[0].name],
                            env=dict(os.environ,ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"],ROS_LOCALHOST_ONLY="1"))
                if operator is not None and operator.poll() is not None:
                    if operator.returncode:
                        raise RuntimeError("The RViz input sequence did not finish successfully")
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
    parser.add_argument("--output",type=Path,default=LOCAL/"release_c3"/"default_runs.json")
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
