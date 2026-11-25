#!/usr/bin/env python3
"""Run sequential Gazebo qualification with the normal launch graph, transport and controller."""
import argparse
from datetime import datetime
import fcntl
import gzip
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from cases import LOCAL, ROOT
from fwis_bringup.experiment_log import collect, remaining_bytes, RUN_BUDGET

# All endpoints are input fixtures, not planner tuning. Long route preserves
# the installed factory layout and scene-defined terminal pose.
FAMILIES = {
    "straight": (3.8, 0., 0.),
    "bend": (3.8, 1.15, .64),
    "lane_change": (4., .7, 0.),
    "diagonal": (3.2, .81, .31),
    "reverse": (-1.45, 0., 0.),
    "cross_dock": (0., 1.1, 0.),
    "close_turn": (1.522379369389306,.34843514176563,.45),
    "long": None,
    "fault": (3.8,0.,0.),
}


def binaries():
    import ament_index_python.packages as pkg
    paths=[]
    for name,nodes in (("fwis_hybrid_joint_ros",["hybrid_joint_planner","hybrid_joint_executor"]),
                       ("fwis_control",["ranger_tracking_controller_cpp"])):
        paths+= [Path(pkg.get_package_prefix(name))/"lib"/name/n for n in nodes]
    for p,lib in (("fwis_hybrid_joint","libfwis_hybrid_joint.so"),("fwis_hybrid_joint_ros","libhybrid_codec.so")):
        paths.append(Path(pkg.get_package_prefix(p))/"lib"/lib)
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def supervise(args, family, task, trial):
    from ranger_mini_gz.scene_geometry import local_pose
    import yaml
    task_arg="multi" if task=="formation" else "single"
    config_args=["--task",task_arg,"--planner-backend","hybrid_joint","--speed-profile",args.profile,
                 "--experiment","full_route" if family=="long" else "short_turn"]
    cfg=yaml.safe_load(subprocess.check_output([ROOT/"scripts/test.sh",*config_args,"--dry-run"],text=True))
    if FAMILIES[family] is not None:
        x,y,th=FAMILIES[family]
        gx,gy,_=local_pose(cfg["start"],(x,y))
        cfg["goal"]=[gx,gy,cfg["start"][2]+th]; cfg["waypoints"]=[]
    cfg["experiment"]="v3_"+family
    cfg["timeout_s"]=args.timeout if args.timeout else (890 if family=="long" else 180)
    cfg["observation_s"]=1.2 if family=="fault" else .3
    run_id=datetime.now().strftime("%Y%m%d_%H%M%S_")+uuid.uuid4().hex[:6]
    archive=ROOT/"reports/runs"/run_id
    if remaining_bytes(archive.parent)<RUN_BUDGET:
        raise RuntimeError("Local experiment storage quota reached")
    archive.mkdir()
    own=args.output/run_id
    own.mkdir()
    row=dict(run_id=run_id,family=family,task=task,trial=trial,profile=args.profile,
             installed_binaries=binaries(),backend=cfg["planner_backend"])
    with tempfile.TemporaryDirectory(prefix="coptrans-v3-gazebo-") as parent:
        spool=Path(parent)/run_id
        for suffix in ["ros","metrics","trajectories","screenshots"]:
            (spool/suffix).mkdir(parents=True,exist_ok=True)
        env=dict(os.environ,ROS_DOMAIN_ID=str(100+os.getpid()%13),GZ_PARTITION="v3_"+run_id,
                 COPTRANS_ARCHIVE_DIR=str(archive),ROS_LOG_DIR=str(spool/"ros"),ROS_LOCALHOST_ONLY="1",
                 COPTRANS_TIMING_DIR=str(own),OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1")
        import ament_index_python.packages as pkg
        env["COPTRANS_TRACE_RCL_TAKE_SO"]=str(Path(pkg.get_package_prefix("fwis_control"))/"lib"/
                                              "fwis_control/libcoptrans_bridge_take_trace.so")
        cfg_path=spool/"resolved_config.yaml"
        cfg_path.write_text(json.dumps(cfg))
        (spool/"environment.txt").write_text("git_commit="+subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()+"\n")
        (spool/"command.txt").write_text(" ".join(sys.argv)+"\n")
        collect(spool,archive)
        print(f"{run_id} {task} {family} #{trial} {args.profile}",flush=True)
        with (spool/"launch.log").open("w") as log:
            ros=subprocess.Popen(["ros2","launch","--noninteractive","fwis_bringup","test.launch.py",
                                  f"run_config:={cfg_path}"],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            observer=subprocess.Popen([sys.executable,str(Path(__file__).with_name("observe_timing.py")),
                                       str(own),"3" if task=="formation" else "1"],env=env)
            fault=None
            try:
                if family=="fault":
                    fault=subprocess.Popen([sys.executable,str(Path(__file__).with_name("inject_stop.py")),
                                            str(own/"fault.json"),"3" if task=="formation" else "1"],env=env)
                try:
                    ros.wait(timeout=cfg["timeout_s"]+45)
                except subprocess.TimeoutExpired:
                    row["launch_watchdog_timeout"]=True
            finally:
                if ros.poll() is None:
                    os.killpg(ros.pid,signal.SIGINT)
                    try:
                        ros.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(ros.pid,signal.SIGKILL)
                        ros.wait()
                observer.terminate();observer.wait(timeout=8)
                if fault and fault.poll() is None:
                    try:
                        fault.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        fault.terminate();fault.wait()
        row["launch_code"]=ros.returncode
        result_file=spool/"result.yaml"
        result=yaml.safe_load(result_file.read_text()) if result_file.exists() else {}
        if not result:
            result={"status":"launch_failed","reason":"no_result"}
            result_file.write_text(yaml.safe_dump(result))
        row.update(status=result["status"],reason=result["reason"])
        row["archive"]=str(archive.relative_to(ROOT))
        collect(spool,archive,final=True)
        for p in own.iterdir():
            if p.suffix in (".csv",".jsonl"):
                p.with_suffix(p.suffix+".gz").write_bytes(gzip.compress(p.read_bytes(),compresslevel=3,mtime=0));p.unlink()
        (own/"metadata.json").write_text(json.dumps(row,indent=2))
        print(f"{run_id} {row['status']} {row['reason']}",flush=True)
        return result


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("--families",default=",".join(FAMILIES)); a.add_argument("--task",choices=["single","formation","both"],default="both")
    a.add_argument("--profile",choices=["baseline","standard"],default="standard")
    a.add_argument("--repetitions",type=int,default=3);a.add_argument("--timeout",type=float)
    a.add_argument("--output",type=Path,required=True);a.add_argument("--stop-on-failure",action="store_true")
    args=a.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    tasks=("single","formation") if args.task=="both" else (args.task,)
    with (ROOT/"reports/runs/.test.lock").open("a") as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for family in args.families.split(","):
            if family not in FAMILIES: a.error("unknown family "+family)
            for task in tasks:
                for i in range(args.repetitions):
                    r=supervise(args,family,task,i)
                    if args.stop_on_failure and r["status"]!=("stopped" if family=="fault" else "passed"):
                        raise SystemExit(1)
if __name__=="__main__":main()
