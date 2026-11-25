#!/usr/bin/env python3
"""Run sequential Gazebo qualification with the normal launch graph, transport and controller."""
import argparse
from dataclasses import replace
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
from qualification_scenes import SPECIAL_SCENES, setup, measured
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
    **dict.fromkeys(SPECIAL_SCENES),
    "fault": (3.8,0.,0.),
}

# Keep the relative maneuver identical for single and formation. At the default
# origin each of these ends inside cnc_01 when carrying the full cargo.
EAST_STAGING_FAMILIES = {"bend", "lane_change", "diagonal"}


def fixture_config(family, task, profile):
    from ranger_mini_gz.scene_geometry import local_pose
    import yaml
    task_arg = "multi" if task == "formation" else "single"
    experiment = "full_route" if family == "long" or family in SPECIAL_SCENES else "short_turn"
    args = [ROOT/"scripts/test.sh", "--task", task_arg, "--planner-backend", "hybrid_joint",
            "--speed-profile", profile, "--experiment", experiment, "--dry-run"]
    if family in SPECIAL_SCENES:
        args.extend(["--scenario", SPECIAL_SCENES[family]])
    cfg = yaml.safe_load(subprocess.check_output(args, text=True))
    if family in EAST_STAGING_FAMILIES:
        cfg["start"] = [20.5, 3.5, 0.]
    if FAMILIES[family] is not None:
        x, y, yaw = FAMILIES[family]
        gx, gy, heading = local_pose(cfg["start"], (x, y))
        cfg["goal"] = [gx, gy, heading + yaw]
    # The solver gets endpoints and the factory map, including on the long route.
    if family not in SPECIAL_SCENES:
        cfg["waypoints"] = []
    return cfg


def fixture_scene(cfg, task, *, paths=None):
    from ranger_mini_gz.scene_generator import load_scene
    scene = load_scene(cfg["scenario"], cfg["scene_variant"], paths=paths)
    # simulation.launch.py applies these same experiment overrides before
    # generating the cargo world and spawning the vehicle bodies.
    experiment = {key: tuple(cfg[key]) for key in ("start", "goal", "waypoints")}
    return replace(scene, experiments={**scene.experiments, task: experiment})


def footprint_margin(cfg, task):
    params = cfg["planner_overrides"]
    if task == "formation":
        return params.get("collision_margin", .05)
    return params.get("vehicle", {}).get("safety_margin", .04)


def fixture_preflight(family, task, cfg, scene):
    from ranger_mini_gz.scene_generator import clearance_summary
    if family in SPECIAL_SCENES:
        return setup(family, task, cfg, scene=scene)
    margin = footprint_margin(cfg, task)

    def inflated_size(size):
        return (size[0]+2*margin, size[1]+2*margin, size[2])

    inflated = replace(scene, vehicle_size=inflated_size(scene.vehicle_size),
                       cargo=replace(scene.cargo, size=inflated_size(scene.cargo.size)))
    spawns = scene.spawn_poses(task)
    proof = {"scene": scene.name, "variant": scene.variant, "safety_margin_m": margin,
             "start_spawn_poses": spawns, "endpoints": {}}
    for key in ("start", "goal"):
        checks = clearance_summary(inflated, task, [(cfg[key], spawns.values() if key == "start" else None)])
        for body, check in checks.items():
            if check["distance_m"] <= 0.:
                raise ValueError(f"{family} {task} {key}: inflated {body} overlaps {check['obstacle']}")
        proof["endpoints"][key] = checks
    return proof


def same_fixture(cfg, archive):
    import yaml
    path = archive / "resolved_config.yaml"
    if not path.exists():
        return False
    previous = yaml.safe_load(path.read_text())
    return all(previous.get(key) == cfg[key] for key in ("scenario", "scene_variant", "start", "goal", "waypoints"))


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
    import yaml
    cfg=fixture_config(family,task,args.profile)
    proof=fixture_preflight(family,task,cfg,fixture_scene(cfg,task))
    cfg["experiment"]="v3_"+family
    cfg["timeout_s"]=args.timeout if args.timeout else (890 if family in ("long","multibend") else 180)
    cfg["gazebo_gui"]=cfg["rviz_gui"]=not args.headless
    cfg["observation_s"]=1.2 if family=="fault" else .3
    run_id=datetime.now().strftime("%Y%m%d_%H%M%S_")+uuid.uuid4().hex[:6]
    archive=ROOT/"reports/runs"/run_id
    if remaining_bytes(archive.parent)<RUN_BUDGET:
        raise RuntimeError("Local experiment storage quota reached")
    archive.mkdir()
    own=args.output/run_id
    own.mkdir()
    row=dict(run_id=run_id,family=family,task=task,trial=trial,profile=args.profile,
             gui=not args.headless,installed_binaries=binaries(),backend=cfg["planner_backend"],scene_proof=proof)
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
        prefix=Path(pkg.get_package_prefix("fwis_hybrid_joint_ros"))
        frozen=prefix.parents[2]/"source_commit"
        installed_sha=(frozen.read_text().strip() if frozen.is_file() else
                       subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip())
        (spool/"environment.txt").write_text(f"git_commit={installed_sha}\nROS_DOMAIN_ID={env['ROS_DOMAIN_ID']}\n")
        row["source_commit"]=installed_sha
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
        if family in SPECIAL_SCENES:
            row["scene_measurements"]=measured(family,task,proof,spool)
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
    a.add_argument("--headless",action="store_true",help="run the actual Gazebo server and DDS graph without GUI clients")
    a.add_argument("--resume",action="store_true",help="skip only completed repetition IDs from this exact executable set")
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
                resume_cfg=fixture_config(family,task,args.profile) if args.resume else None
                for i in range(args.repetitions):
                    if (args.output/"STOP_AFTER_TRIAL").exists():
                        print("Current trial finished; yielding the simulator to the next build.",flush=True)
                        raise SystemExit(75)
                    if args.resume:
                        binaries_now=binaries()
                        done=False
                        for meta in args.output.glob("*/metadata.json"):
                            entry=json.loads(meta.read_text())
                            if (entry["family"],entry["task"],entry["trial"],entry["profile"],entry.get("gui",True)) != (family,task,i,args.profile,not args.headless):
                                continue
                            if entry["installed_binaries"]!=binaries_now:
                                raise ValueError("Existing trial used other binaries; select a fresh output directory.")
                            if same_fixture(resume_cfg, ROOT/entry["archive"]):
                                done=True
                        if done:
                            continue
                    r=supervise(args,family,task,i)
                    if args.stop_on_failure and r["status"]!=("stopped" if family=="fault" else "passed"):
                        raise SystemExit(1)
if __name__=="__main__":main()
