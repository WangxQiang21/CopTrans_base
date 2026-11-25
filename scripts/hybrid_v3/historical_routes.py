#!/usr/bin/env python3
"""Replay selected physical endpoints on their archived factory maps through the native planner."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from functools import lru_cache
from itertools import groupby
from contextlib import nullcontext
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

import yaml

from benchmark import invoke
from cases import ROOT, digest, packed
from ranger_mini_gz.scene_generator import read_map, write_map
from ranger_mini_gz.scene_world import write_world

PUBLIC_ROUTES = (
    "20261125_115519_smXjHv", "20261125_120655_6xtU9x",
    "20261125_002559_WxQnZe", "20261125_004431_1tfhhn",
    "20261125_013436_4DilHE", "20261125_014504_A5RlL6",
    "20261125_015146_OS7Yaz", "20261125_015310_79b145",
    "20261125_132051_mGuofk", "20261125_132257_XKwYLG",
    "20261125_121644_JXQ8SY", "20261125_115245_qGa7co", "20261125_004851_QZBuhe",
)


def file_sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return yaml.safe_load(path.read_bytes())


def _route(archive_id, *, archive_root=None):
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", archive_id):
        raise ValueError("archive_id must identify one local run")
    run = (archive_root or ROOT / "reports/runs") / archive_id
    manifest = load(run/"experiment.json")
    source_cfg = load(run/"resolved_config.yaml")
    original = load(run/"result.yaml")
    meta = load(run/"scene/scene_meta.yaml")
    if manifest["experiment_id"]!=archive_id or file_sha(run/"scene/scene_meta.yaml")!=manifest["scene_sha256"]:
        raise ValueError(f"historical archive identity mismatch: {archive_id}")
    selected = run/"selected_endpoints.yaml"
    endpoints = load(selected) if selected.exists() else manifest
    for key in ("start","goal"):
        if endpoints.get(key)!=manifest.get(key) or len(endpoints[key])!=3 or not all(map(math.isfinite,endpoints[key])):
            raise ValueError(f"{archive_id} inconsistent {key}")
    pgm = gzip.decompress((run/"scene/map.pgm.gz").read_bytes())
    if hashlib.sha256(pgm).hexdigest()!=manifest["map_sha256"]:
        raise ValueError(f"{archive_id} stored map does not match manifest")
    return run, manifest, source_cfg, original, meta, endpoints


def historical_fixture(archive_id, task, profile, *, directory=None, archive_root=None):
    """Return resolved C6 config, physical scene with the actual spawn, and a verified provenance receipt."""
    from closed_loop import fixture_scene, fixture_preflight
    if task not in ("single","formation") or profile not in ("standard","baseline"):
        raise ValueError("unsupported historical task or profile")
    run, manifest, src, old, meta, endpoints = _route(archive_id, archive_root=archive_root)
    scenario, variant = meta["scenario"], meta["variant"]
    args = [ROOT/"scripts/test.sh","--dry-run","--planner-backend","hybrid_joint","--task",
            "multi" if task=="formation" else "single","--speed-profile",profile,"--scenario",scenario,
            "--scene-variant",variant,"--experiment","full_route"]
    cfg = yaml.safe_load(subprocess.check_output(args,text=True))
    cfg.update(start=endpoints["start"], goal=endpoints["goal"], waypoints=[],
               experiment="historical_"+archive_id, historical_source_id=archive_id,
               replay_map_sha256=manifest["map_sha256"])
    scene = fixture_scene(cfg, task)
    objects = [o.name for o in scene.obstacles if o.map_obstacle]
    if set(objects)!=set(meta["environment_obstacles"]):
        raise ValueError(f"{archive_id} obstacle entities differ from archived world")
    archive_grid = load(run/"scene/map.yaml")
    with TemporaryDirectory(prefix="coptrans-history-") if directory is None else nullcontext(directory) as scratch:
        output = Path(scratch)
        output.mkdir(parents=True, exist_ok=True)
        write_map(scene, output, "map", scene.resolution)
        write_world(scene, output/"world.sdf", task)
        generated = load(output/"map.yaml")
        if generated!=archive_grid or file_sha(output/"map.pgm")!=manifest["map_sha256"]:
            raise ValueError(f"{archive_id} regenerated {scenario}/{variant} map differs from original")
        world = ElementTree.parse(output/"world.sdf").find("world")
        world_entities = {m.attrib["name"] for m in world.findall("model")}
        if world.attrib["name"]!=scenario or set(objects)-world_entities:
            raise ValueError(f"{archive_id} generated SDF does not contain each mapped obstacle")
        world_hash = file_sha(output/"world.sdf")
    spawns = scene.spawn_poses(task)
    same_task = src["scene_task"]==task
    measured = old.get("observed_spawn") or {}
    spawn_agrees = None
    if same_task and measured:
        spawn_agrees = all(math.dist(pos[:2],measured[i][:2])<=.003 and
                           abs(math.remainder(pos[2]-measured[i][2],2*math.pi))<=.003 for i,pos in spawns.items())
        if not spawn_agrees:
            raise ValueError(f"{archive_id}: recorded initial vehicle poses differ from generated spawns")
    observed_source = {str(i):v for i,v in measured.items()}
    if task=="formation" and (scene.offsets!=tuple(tuple(x) for x in meta["formation_offsets_m"]) or
                              scene.cargo.size!=tuple(meta["cargo"]["size_m"]) or scene.cargo.offset!=tuple(meta["cargo"]["offset_m"])):
        raise ValueError(f"{archive_id} mount/cargo geometry changed")
    old_params = src.get("planner_overrides", {})
    old_timing = old_params.get("timing_limits") if src["task"]=="multi" else old_params.get("quality",{}).get("limits")
    fresh = cfg["planner_overrides"]
    timing = fresh.get("timing_limits") if task=="formation" else fresh.get("quality",{}).get("limits")
    old_timing_matches = same_task and bool(old_timing) and all(timing.get(k)==v for k,v in old_timing.items())
    proof = dict(archive_id=archive_id, scenario=scenario, variant=variant, start=cfg["start"], goal=cfg["goal"],
                 task=task, profile=profile, source_status=manifest["status"], source_reason=manifest.get("reason"),
                 source_profile=src["speed_profile"], source_task=src["scene_task"], source_backend=src.get("planner_backend"),
                 source_commit=manifest.get("git_commit"), archive_scene_meta_sha256=manifest["scene_sha256"],
                 source_plan_duration_s=old.get("plan_duration_s"), source_planner_wall_s=old.get("planner_wall_s"),
                 map_sha256=manifest["map_sha256"], map_pgm_sha256=manifest["map_sha256"], generated_world_sha256=world_hash,
                 stored_world_sdf_available=(run/"scene/world.sdf").is_file(), map_resolution=archive_grid["resolution"],
                 map_origin=archive_grid["origin"], obstacle_ids=objects, observed_source_spawn=observed_source,
                 generated_spawn={str(i):v for i,v in spawns.items()}, source_spawn_matches=spawn_agrees,
                 same_task=same_task, same_profile=profile==src["speed_profile"], old_timing_limits=old_timing,
                 old_timing_matches=bool(old_timing_matches), controller_overrides_match=cfg["controller_limit_overrides"]==src["controller_limit_overrides"],
                 plant_overrides_match=cfg["plant_overrides"]==src["plant_overrides"], cargo_size=list(scene.cargo.size),
                 cargo_offset=scene.cargo.offset, formation_offsets=scene.offsets, vehicle_size=scene.vehicle_size)
    try:
        proof["endpoint_clearance"] = fixture_preflight("historical",task,cfg,scene)
    except ValueError as error:
        proof["endpoint_clearance_error"] = str(error)
    return cfg, scene, proof


@lru_cache(maxsize=None)
def original_plan(archive_id):
    from fwis_v3_msgs.msg import FormationPlaybackPlan, TimedTrajectory
    from rclpy.serialization import deserialize_message
    run, manifest, cfg, result, _, _ = _route(archive_id)
    trajectory = manifest.get("plan_artifact")
    metrics = result.get("metrics") or {}
    info = dict(source_cdr=None, sampled_reference_length_m=None, sample_count=0,
                sampled_modes_by_vehicle={}, sampled_switches_by_vehicle={},
                max_measured_mount_error_m=metrics.get("max_mount_error_m"),
                terminal_formation_error_m=metrics.get("terminal_formation_position_error_m"))
    if trajectory is None:
        return info
    source = run / trajectory
    cdr = gzip.decompress(source.read_bytes())
    multi = cfg["scene_task"]=="formation"
    msg = deserialize_message(cdr, FormationPlaybackPlan if multi else TimedTrajectory)
    poses = msg.trajectory.formation_poses if multi else [p.center_pose for p in msg.points]
    tracks = {str(t.vehicle_id): t.points for t in msg.trajectory.trajectories} if multi else {"1":msg.points}
    modes = {i: ["AOS"[kind] for kind, _ in groupby(int(p.mode) for p in points)] for i, points in tracks.items()}
    info.update(source_cdr=trajectory, cdr_sha256=hashlib.sha256(cdr).hexdigest(), sample_count=len(poses),
                sampled_reference_length_m=sum(math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(poses,poses[1:])),
                sampled_modes_by_vehicle=modes, sampled_switches_by_vehicle={i:len(m)-1 for i,m in modes.items()},
                sampled_start=[poses[0].x,poses[0].y,poses[0].theta], sampled_goal=[poses[-1].x,poses[-1].y,poses[-1].theta])
    return info


def save(path, data):
    temp = path.with_suffix(path.suffix+".tmp")
    temp.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False,indent=2)+"\n")
    temp.replace(path)


def identify_build(executable, source_commit):
    linked = subprocess.check_output(["ldd", executable], text=True)
    lib = Path(next(line.split("=>",1)[1].split(" (",1)[0].strip() for line in linked.splitlines()
                    if "libfwis_hybrid_joint.so =>" in line))
    core = subprocess.check_output(["git","log","-1","--format=%H",source_commit,"--","ros2_ws/src/fwis_hybrid_joint"],
                                   cwd=ROOT, text=True).strip()
    policy = subprocess.check_output(["git","show",source_commit+":ros2_ws/src/fwis_hybrid_joint/config/policy.json"],cwd=ROOT)
    return dict(source_commit=source_commit,core_commit=core,binary_sha256=file_sha(executable),library_sha256=file_sha(lib),
                policy_sha256=hashlib.sha256(policy).hexdigest(), executable=str(executable), library=str(lib))


def quaternion_yaw(yaw):
    # Same planar PoseStamped encoding and decoding as the ordinary ROS interfaces.
    z, w = math.sin(yaw / 2), math.cos(yaw / 2)
    return math.atan2(2 * w * z, 1 - 2 * z * z)


def ros_grid_sha256(grid):
    # JsonCpp StreamWriter at precision 17 spells integral doubles with a decimal.
    def json_double(v):
        number = format(v, ".17g")
        return number if "." in number or "e" in number else number + ".0"

    meta = [grid["width"], grid["height"], grid["resolution"], *grid["origin"]]
    data = bytearray(grid["width"] * grid["height"])
    for offset, length in grid["occupied_runs"]:
        data[offset:offset+length] = bytes([100]) * length
    header = "[" + ",".join(map(json_double,meta)) + "]:"
    return hashlib.sha256(header.encode() + data).hexdigest()


def ros_contract(req, formation_id):
    multi = len(req["vehicles"]) == 3
    contract = dict(id=1, formation_id=formation_id if multi else "single", vehicle_ids=[1,2,3] if multi else [1],
                    grid=ros_grid_sha256(req["grid"]), occupancy="nonzero_or_outside_occupied")
    for k in ("allow_reverse","cargo","limits","task_duration_cap","vehicles"):
        contract[k]=req[k]
    for key in ("start","goal"):
        q = req[key]
        contract[key] = dict(reference=q["reference"], heading=q["heading"],
                             headings_constrained=q["constrain_heading"],
                             reference_yaw_constrained=q["constrain_reference_yaw"])
    return contract


def native_request(cfg, scene, task, grid, proof, *, goal=None):
    """Mirror initialize_planner/publish_goal and the native ROS request contract."""
    from fwis_bringup.formation_route import fit_reference
    from scene_benchmark import request_from_scene
    selected = cfg if goal is None else {**cfg, "goal":goal}
    req = request_from_scene(scene,task,selected,grid)
    multi = task == "formation"
    if proof["same_task"] and proof["observed_source_spawn"]:
        spawns = [tuple(p) for _,p in sorted(proof["observed_source_spawn"].items())]
    else:
        spawns = [(x,y,quaternion_yaw(yaw)) for x,y,yaw in scene.spawn_poses(task).values()]
    if multi:
        req["start"]["reference"] = list(fit_reference(scene.offsets, spawns))
        req["start"]["heading"] = [p[2] for p in spawns]
    else:
        x,y,yaw = spawns[0]
        req["start"]["reference"] = [x,y,quaternion_yaw(yaw)]
        req["start"]["heading"] = [req["start"]["reference"][2], 0., 0.]
        x,y,yaw = req["goal"]["reference"]
        req["goal"]["reference"] = [x,y,quaternion_yaw(yaw)]
        req["goal"]["heading"] = [req["goal"]["reference"][2], 0., 0.]
    params = cfg["planner_overrides"]
    timing = params["timing_limits"] if multi else params["quality"]["limits"]
    req["limits"]["min_motion_duration"] = timing.get("min_mode_dwell_s",.30)
    return req


def replay_request(archive, task, profile, goal=None):
    """Build the same blind numerical request as the batch runner from a local log."""
    archive = Path(archive).resolve()
    with TemporaryDirectory(prefix="coptrans-history-request-") as tmp:
        folder = Path(tmp)
        cfg, scene, proof = historical_fixture(archive.name,task,profile,directory=folder,archive_root=archive.parent)
        req = native_request(cfg,scene,task,read_map(folder/"map.yaml"),proof,goal=goal)
    return dict(operation="solve",request=req,budget_s=cfg["planner_overrides"]["solve_budget_s"])


def prepare(archive_id, task, profile, output):
    base = output / "archives" / archive_id
    scene_dir = base / task
    cfg, scene, proof = historical_fixture(archive_id,task,profile,directory=scene_dir)
    proof["source_plan"] = original_plan(archive_id)
    stem = base / f"{task}_{profile}"
    req = native_request(cfg,scene,task,read_map(scene_dir/"map.yaml"),proof)
    contract = ros_contract(req,scene.formation_id)
    save(stem.with_suffix(".ros_contract.json"),contract)
    budget = cfg["planner_overrides"]["solve_budget_s"]
    payload = dict(operation="solve", request=req, budget_s=budget)
    save(stem.with_suffix(".config.json"),cfg)
    stem.with_suffix(".request.json").write_bytes(packed(payload)+b"\n")
    row = dict(archive_id=archive_id,task=task,profile=profile,request_sha256=digest(req),map_sha256=proof["map_sha256"],
               start=cfg["start"],goal=cfg["goal"],budget_s=budget,request_duration_cap_s=req["task_duration_cap"],
               ros_contract_sha256=digest(contract), ros_grid_sha256=contract["grid"],
               expected_backend=cfg["planner_backend"],proof=proof,archive_stem=str(stem.relative_to(output)))
    return payload,row


def solve_case(entry):
    executable,payload,row,stem = entry
    solved = invoke(executable,payload,payload["budget_s"]+10)
    stem.with_suffix(".result.json").write_bytes(packed(solved)+b"\n")
    audit = {}
    if solved.get("plan"):
        audit = invoke(executable,dict(operation="certify",request=payload["request"],plan=solved["plan"],budget_s=8),15)
    stem.with_suffix(".audit.json").write_bytes(packed(audit)+b"\n")
    metrics = solved.get("metrics",{})
    row.update(actual_backend=solved.get("actual_backend"), status=solved.get("status"), reason=solved.get("reason"),
               modes=metrics.get("mode_sequence"), duration_s=solved.get("duration_s"),solve_wall_s=solved.get("wall_s"),
               reference_length_m=metrics.get("policy_reference_length_m"),solver_metrics=metrics, audit=audit,
               first_certified_s=solved.get("first_certified_s"), solver_audit=solved.get("audit"),
               stage_times=solved.get("stage_times"),candidate_trace=solved.get("trace",[]),plan_sha256=digest(solved["plan"]) if solved.get("plan") else None,
               passed=solved.get("status")=="CERTIFIED_POLICY_COMPLIANT" and audit.get("executable") is True and
                      solved.get("actual_backend")==row["expected_backend"])
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id",action="append",dest="ids",help="local archived experiment ID; repeat to select a cohort")
    parser.add_argument("--task",choices=("formation","single","both"),default="both")
    parser.add_argument("--profile",choices=("standard","baseline","both"),default="both")
    parser.add_argument("--output",type=Path,required=True,help="new local evidence directory")
    parser.add_argument("--executable",type=Path,default=ROOT/"reports/acceptance/hybrid_v3/system_c6/ros2_ws/install/fwis_hybrid_joint/lib/fwis_hybrid_joint/hybrid_joint_case")
    parser.add_argument("--source-commit",default="c09357a32ceab3ca9870daeed6b4e867405ea5c0",help="commit of the supplied native binary")
    parser.add_argument("--jobs",type=int,default=1)
    args=parser.parse_args()
    if args.jobs<1:
        parser.error("--jobs must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    executable = args.executable.resolve()
    ids = list(dict.fromkeys(args.ids or PUBLIC_ROUTES))
    tasks = ("formation","single") if args.task=="both" else (args.task,)
    profiles = ("standard","baseline") if args.profile=="both" else (args.profile,)
    summary = identify_build(executable,args.source_commit)
    summary.update(generated_utc=datetime.now(timezone.utc).isoformat(),complete=False,requested_ids=ids,
                   fixture_source_sha256=file_sha(Path(__file__)),cases=[],duplicate_requests=[])
    known = {}
    pending = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for route in ids:
            for task in tasks:
                for profile in profiles:
                    payload,row=prepare(route,task,profile,output)
                    key = (row["request_sha256"],row["budget_s"],row["expected_backend"])
                    if key in known:
                        summary["duplicate_requests"].append(dict(archive_id=route,task=task,profile=profile,
                                                                request_sha256=key[0],same_as=known[key]))
                        continue
                    known[key]=row["archive_stem"]
                    pending.append(pool.submit(solve_case,(executable,payload,row,output/row["archive_stem"])))
            ready=[f for f in pending if f.done()]
            for future in ready:
                row=future.result()
                summary["cases"].append(row)
                print(row["archive_id"],row["task"],row["profile"],row["status"],row["modes"],flush=True)
                pending.remove(future)
            save(output/"summary.json",summary)
        for future in pending:
            row=future.result()
            summary["cases"].append(row)
            print(row["archive_id"],row["task"],row["profile"],row["status"],row["modes"],flush=True)
            save(output/"summary.json",summary)
    summary.update(complete=True,passed=sum(r["passed"] for r in summary["cases"]),unique_requests=len(known))
    save(output/"summary.json",summary)
    return 0 if summary["passed"]==len(known) else 1


if __name__=="__main__":
    raise SystemExit(main())
