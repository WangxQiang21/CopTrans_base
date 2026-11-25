#!/usr/bin/env python3
"""Recompute section 17.8 from archived Bernstein plans and optional Gazebo odometry."""
import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
from math import comb
from pathlib import Path

import numpy as np
import yaml

from cases import ROOT, digest

CONTROL = ROOT/"ros2_ws/src/fwis_control/config/ranger_tracking.yaml"
POLICY = ROOT/"ros2_ws/src/fwis_hybrid_joint/config/policy.json"
SAMPLE_S = .01


def mode(phase):
    m = phase["mode"]
    return "AOS"[m] if isinstance(m, int) else m


def motion(phase):
    return phase["kind"] in (0, "motion")


def coefficients(phase):
    return np.asarray([[p[k] for k in ("x", "y", "yaw")] if isinstance(p, dict) else p for p in phase["pieces"]])


def station(t, ramp):
    if not ramp:
        h = np.minimum(t, 1-t)
        s = h**3*(10+h*(-15+6*h))
        return np.where(t <= .5, s, 1-s), 30*t**2*(1-t)**2
    h = np.minimum(t, 1-t)/ramp
    s = ramp*h**3*(1-.5*h)/(1-ramp)
    cruise = (t > ramp) & (t < 1-ramp)
    return (np.where(cruise, (t-.5*ramp)/(1-ramp), np.where(t <= .5, s, 1-s)),
            np.where(cruise, 1., h*h*(3-2*h))/(1-ramp))


def spatial(curve, s, offsets):
    n = len(curve)
    index = np.minimum((s*n).astype(int), n-1)
    u = s*n-index
    controls = curve[index]
    jets = []
    for degree in (5, 4, 3):
        basis = np.array([comb(degree,k)*u**k*(1-u)**(degree-k) for k in range(degree+1)]).T
        jets.append(np.einsum("tac,tc->ta", controls, basis))
        controls = degree*n*np.diff(controls, axis=2)
    pos, vel, acc = [v[:, :, None] for v in jets]
    c, h = np.cos(pos[:, 2]), np.sin(pos[:, 2])
    rx, ry = c*offsets[:, 0]-h*offsets[:, 1], h*offsets[:, 0]+c*offsets[:, 1]
    dx, dy = vel[:, 0]-vel[:, 2]*ry, vel[:, 1]+vel[:, 2]*rx
    ax = acc[:, 0]-acc[:, 2]*ry-vel[:, 2]**2*rx
    ay = acc[:, 1]+acc[:, 2]*rx-vel[:, 2]**2*ry
    norm = np.hypot(dx, dy)
    curvature = np.divide(dx*ay-dy*ax, norm**3, out=np.zeros_like(norm), where=norm>1e-12)
    return jets[0][:, :2], dx, dy, norm, curvature


def wheel_positions(req):
    limits = req["limits"]
    return np.asarray([(x*limits["wheelbase"]/2, y*limits["track"]/2)
                       for x in (1., -1.) for y in (1., -1.)])


def wrap_axis(a):
    return (a + np.pi/2) % np.pi - np.pi/2


def stop_intervals(times, speed, threshold, window):
    slow = speed < threshold
    if len(times)<3:
        return []
    starts = np.flatnonzero(np.diff(np.r_[False, slow].astype(int))==1)
    ends = np.flatnonzero(np.diff(np.r_[slow, False].astype(int))==-1)
    intervals = []
    for a, b in zip(starts, ends):
        if a==0 or b==len(speed)-1:
            continue
        # Linear interpolation of the two threshold crossings on the declared time grid.
        entry = np.interp(threshold, [speed[a], speed[a-1]], [times[a], times[a-1]])
        exit_ = np.interp(threshold, [speed[b], speed[b+1]], [times[b], times[b+1]])
        if exit_-entry >= window:
            intervals.append((entry, exit_))
    return intervals


def plan_metrics(req, phases, timing, *, trace=(), attempts=None, initial_sequence=None):
    offsets = np.asarray(req["vehicles"])[:, :2]
    wheels = wheel_positions(req)
    count = len(offsets)
    zero = json.loads(POLICY.read_text())["zero_motion_tolerance"]
    stop = timing["stop_speed"]
    window = max(timing["stable_time"], 2/timing["frequency"])
    peaks = np.zeros((count, 4))
    jumps = np.zeros(count)
    blocks = []
    start = 0.
    for ph in phases:
        dur = ph["duration"]
        if not motion(ph):
            start += dur
            continue
        curve, m = coefficients(ph), mode(ph)
        gear = tuple(ph["gear"][:count])
        n = max(32*len(curve), int(np.ceil(dur/SAMPLE_S)))
        t = np.linspace(0., 1., n+1)
        s, ds = station(t, ph.get("cruise_ramp_fraction", 0))
        xy, dx, dy, norm, cur = spatial(curve, s, offsets)
        distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        spin = np.abs(np.array(ph["heading1"][:count])-ph["heading0"][:count]) if m=="S" else np.zeros(count)
        if distance[-1] <= zero and sum(spin) <= zero:
            start += dur
            continue
        if m=="A":
            cur *= gear
            angles = np.arctan2(cur[:, :, None]*wheels[:, 0], 1-cur[:, :, None]*wheels[:, 1])
            if len(curve)>1:
                knots = np.arange(1,len(curve))/len(curve)
                left = spatial(curve, knots-1e-12, offsets)[-1]
                right = spatial(curve, knots+1e-12, offsets)[-1]
                jumps = np.maximum(jumps, np.max(np.abs(left-right), axis=0))
        elif m=="O":
            heading = np.asarray(ph["heading0"][:count])
            angle = np.arctan2(gear*(-np.sin(heading)*dx+np.cos(heading)*dy),
                               gear*(np.cos(heading)*dx+np.sin(heading)*dy))
            angles = np.repeat(angle[:, :, None], 4, axis=2)
        else:
            angles = np.tile(wrap_axis(np.arctan2(wheels[:, 0], -wheels[:, 1])), (1, count, 1))
        peaks = np.maximum(peaks, np.abs(angles).max(axis=0))
        speed = norm*ds[:, None]/dur
        if blocks and (blocks[-1]["mode"], blocks[-1]["gear"]) == (m, gear):
            block = blocks[-1]
            if m=="A":
                jumps = np.maximum(jumps, np.abs(block["end_curvature"]-cur[0]))
            prior_len = block["length"]
            block["times"].extend(start+t*dur)
            block["speeds"].extend(speed)
            block["arc"].extend(prior_len+distance)
            block["length"] += float(distance[-1])
            block["spin"] += spin
            block["end_curvature"] = cur[-1]
        else:
            blocks.append(dict(mode=m, gear=gear, times=list(start+t*dur), speeds=list(speed),
                               arc=list(distance), length=float(distance[-1]), spin=spin, end_curvature=cur[-1]))
        start += dur
    internal = 0
    individual_stops = np.zeros(count, dtype=int)
    longest = 0.
    roles = []
    for i, b in enumerate(blocks):
        if b["mode"]!="A":
            role = "sole" if len(blocks)==1 else "initial" if i==0 else "terminal" if i==len(blocks)-1 else "interior"
            roles.append(dict(mode=b["mode"], role=role, length_m=b["length"], rotation_rad_by_vehicle=b["spin"].tolist()))
            continue
        times, speeds = np.array(b["times"]), np.array(b["speeds"])
        for v in range(count):
            individual_stops[v] += len(stop_intervals(times, speeds[:, v], stop, window))
        events = stop_intervals(times, speeds.max(axis=1), stop, window)
        internal += len(events)
        stops = [times[0], *[(a+c)/2 for a,c in events], times[-1]]
        longest = max(longest, float(np.max(np.diff(np.interp(stops, times, b["arc"])))))
    seq = "".join(b["mode"] for b in blocks)
    a = sum(b["length"] for b in blocks if b["mode"]=="A")
    o = sum(b["length"] for b in blocks if b["mode"]=="O")
    raw_seq = "".join(mode(p) for p in phases if motion(p))
    # A certificate alone does not identify the incumbent when multiple seeds
    # with identical structure were certified. Preserve that ambiguity explicitly.
    tags = [line.split(":") for line in trace if line.endswith(":CERTIFIED")]
    same = [":".join(tag[:3]) for tag in tags if len(tag)>=4 and tag[1]==raw_seq]
    if blocks:
        signed = [tag for tag in same if tag.rsplit(":",1)[-1]==str(blocks[0]["gear"][0])]
        same = signed or same
    certified = {":".join(tag[:3]) for tag in tags if len(tag)>=4}
    # A sole certified proposal is definitive even if expansion eliminated
    # a physically empty S/O phase inside its original structure.
    sources = sorted(certified if len(certified)==1 else set(same))
    first = initial_sequence or (trace[0].split(":")[1] if trace and len(trace[0].split(":"))>=3 else None)
    edits = ("INSERT_MODE", "REMOVE_BLOCK", "MERGE_SAME_MODE", "REPLACE_SUBSEQUENCE", "CHANGE_GEAR")
    edits_tried = None if attempts is None else Counter(a["proposal"].split(":")[0] for a in attempts
                                                        if a["proposal"].split(":")[0] in edits)
    return dict(raw_modes=raw_seq, normalized_modes=seq, pure_A=bool(seq) and set(seq)=={"A"},
                A_translation_fraction=a/(a+o) if a+o>zero else None,
                A_length_m=a, O_length_m=o, S_angle_rad=sum(sum(b["spin"]) for b in blocks),
                S_blocks=seq.count("S"), aux_blocks=len(roles), auxiliary_roles=roles,
                longest_continuous_A_m=longest, internal_stops=internal, stops_by_vehicle=individual_stops.tolist(),
                curvature_jump_per_vehicle_m_inv=jumps.tolist(), plan_peak_abs_wheel_angle_rad=peaks.tolist(),
                initial_mode_proposal=first, net_motion_block_reduction=len(first)-len(seq) if first is not None else None,
                net_aux_block_reduction=sum(c in "OS" for c in first)-len(roles) if first is not None else None,
                structural_edits_attempted=edits_tried,
                winning_proposal=sources[0] if len(sources)==1 else None, matching_certified_proposals=sources,
                stop_speed_mps=stop, stop_window_s=window)


def rolling_axis(twist, wheel_xy):
    vx, vy, wz = twist
    wx, wy = vx-wz*wheel_xy[:, 1], vy+wz*wheel_xy[:, 0]
    return np.abs(wrap_axis(np.arctan2(wy, wx))), np.hypot(wx, wy)>1.e-6


def measured_wheels(archive, req, plant_folder):
    path = archive/"metrics/motion.jsonl.gz"
    if not path.exists():
        return None
    offsets = wheel_positions(req)
    peak = np.zeros((len(req["vehicles"]), 4))
    observed = np.zeros(len(peak), dtype=int)
    stamps = []
    with gzip.open(path, "rt") as stream:
        for line in stream:
            row = json.loads(line)
            stamps.append(row["stamp_s"])
            for key, twist in row.get("odom_twist", {}).items():
                v = int(key)-1
                if v<0 or v>=len(peak):
                    raise ValueError("unexpected vehicle ID in measured odometry")
                angles, defined = rolling_axis(twist, offsets)
                if np.any(defined):
                    observed[v] += 1
                    peak[v, defined] = np.maximum(peak[v, defined], angles[defined])
    applied = []
    if plant_folder:
        for v in range(len(peak)):
            plant = plant_folder/f"robot{v+1}_plant_timing.csv.gz"
            if not plant.exists():
                applied.append(None)
                continue
            max_command = np.zeros(4)
            count = 0
            with gzip.open(plant, "rt") as stream:
                for row in csv.DictReader(stream):
                    if row["event"]=="command" and row["applied_steady_ns"]:
                        twist = tuple(float(row[k]) for k in ("vx_mps","vy_mps","wz_radps"))
                        angles, defined = rolling_axis(twist, offsets)
                        if np.any(defined):
                            max_command[defined] = np.maximum(max_command[defined], angles[defined])
                            count += 1
            applied.append(dict(moving_applied_commands=count, implied_peak_abs_wheel_angle_rad=max_command.tolist() if count else None))
    return dict(odom_implied_peak_abs_wheel_angle_rad=[angles.tolist() if n else None for n, angles in zip(observed, peak)],
                moving_odom_samples=observed.tolist(), odom_samples=len(stamps),
                max_odom_sample_gap_s=float(np.diff(stamps).max()) if len(stamps)>1 else None,
                applied_twist_by_vehicle=applied, wheel_angle_sensor_samples=0)


def scene_rows(folder, timing):
    run = json.loads((folder/"summary.json").read_text())
    for row in run["cases"]:
        case = {k: row[k] for k in ("family","task","profile","request_sha256","status","passed")}
        name = folder/row["family"]/(row["task"]+"_"+row["profile"])
        req = json.loads(name.with_suffix(".request.json").read_bytes())["request"]
        result = json.loads(name.with_suffix(".result.json").read_bytes())
        case["independently_certified"] = row.get("audit", {}).get("executable") is True
        if digest(req)!=row["request_sha256"]:
            raise ValueError(f"archive request mismatch for {name}")
        if result.get("plan"):
            case["plan_sha256"] = digest(result["plan"])
            case["metrics"] = plan_metrics(req, result["plan"], timing, trace=result["trace"],
                                           attempts=result.get("stage_times", {}).get("attempts"))
        yield case


def run_row(folder, plant_folder=None):
    manifest = json.loads((folder/"experiment.json").read_text())
    cfg = yaml.safe_load((folder/"resolved_config.yaml").read_text())
    result = yaml.safe_load((folder/"result.yaml").read_text())
    row = dict(run_id=folder.name, task=cfg["scene_task"], profile=cfg["speed_profile"],
               family=cfg["experiment"], status=result["status"], source_commit=manifest.get("git_commit"))
    if plant_folder:
        meta = json.loads((plant_folder/"metadata.json").read_text())
        if meta["run_id"]!=folder.name or meta["status"]!=row["status"]:
            raise ValueError(f"case metadata differs from archive {folder}")
        row.update(family=meta["family"], trial=meta["trial"], gui=meta["gui"], qualification_group=plant_folder.parent.name)
    measured = result.get("metrics") or {}
    vehicles = measured.get("vehicles") or {}
    row["measured_geometry"] = dict(
        terminal_position_error_m_by_vehicle={str(i):v.get("terminal_position_error_m") for i,v in vehicles.items()},
        terminal_yaw_error_deg_by_vehicle={str(i):v.get("terminal_yaw_error_deg") for i,v in vehicles.items()},
        max_mount_error_m=measured.get("max_mount_error_m"),
        terminal_reference_position_error_m=measured.get("terminal_formation_position_error_m",measured.get("requested_goal_position_error_m")),
        terminal_reference_yaw_error_deg=measured.get("terminal_formation_yaw_error_deg",measured.get("requested_goal_yaw_error_deg")),
        min_body_gap_m=measured.get("min_vehicle_footprint_gap_m"),
        min_environment_clearance_m={i:g["m"] for i,g in measured.get("min_environment_clearance",{}).items()})
    model = folder/"trajectories/model.json.gz"
    if model.exists():
        data = gzip.decompress(model.read_bytes())
    elif cfg["scene_task"]=="formation" and (folder/"trajectories/planner.cdr.gz").exists():
        from fwis_v3_msgs.msg import FormationPlaybackPlan
        from rclpy.serialization import deserialize_message
        packed = gzip.decompress((folder/"trajectories/planner.cdr.gz").read_bytes())
        data = deserialize_message(packed, FormationPlaybackPlan).model_json.encode()
    else:
        row["reason"] = "authoritative_model_unavailable"
        return row
    model_hash = hashlib.sha256(data).hexdigest()
    if manifest.get("model_sha256") and manifest["model_sha256"] != model_hash:
        raise ValueError(f"archive authority digest mismatch for {folder}")
    model = json.loads(data)
    if "phases" not in model:
        row["reason"] = "archived_model_uses_a_different_backend"
        return row
    req = model["request"]
    timing = yaml.safe_load((folder/"controller_parameters.yaml").read_text())["timing"]
    row.update(plan_sha256=model_hash, metrics=plan_metrics(req, model["phases"], timing),
               wheel_odometry=measured_wheels(folder, req, plant_folder))
    return row


def bench_rows(files, cohort, timing):
    from cases import MASTER_SEEDS, make
    MASTER_SEEDS["holdout"] += cohort*1000003
    try:
        cache = {}
        metrics_cache = {}
        for path in files:
            with path.open() as stream:
                for line in stream:
                    item = json.loads(line)
                    if item["operation"] not in ("solve","ablation","timing"):
                        continue
                    result = item["evidence"]
                    row = {k:item[k] for k in ("task","profile","case_id","label","repeat","ablation","operation","request_hash",
                                               "dataset_split","source_sha","exe_hash","lib_hash","ranking_policy_sha256")}
                    row["status"] = result["status"]
                    row["independently_certified"] = result.get("audit", {}).get("executable") is True
                    key = item["request_hash"]
                    if key not in cache:
                        req, _, _ = make(item["dataset_split"], item["index"], item["task"], item["profile"])
                        if digest(req)!=key:
                            raise ValueError(f"case regeneration mismatch at {item['case_id']}")
                        cache[key] = {k:req[k] for k in ("vehicles","limits")}
                    if result.get("plan"):
                        row["plan_sha256"] = digest(result["plan"])
                        attempts = result.get("stage_times",{}).get("attempts", [])
                        identity = (key,row["plan_sha256"],tuple(result["trace"]),tuple(a["proposal"] for a in attempts),bool(item["ablation"]))
                        if identity not in metrics_cache:
                            metrics_cache[identity] = plan_metrics(cache[key], result["plan"], timing, trace=result["trace"],
                                                                  attempts=attempts,initial_sequence="ASOSA" if item["ablation"] else None)
                        row["metrics"] = metrics_cache[identity]
                    yield row
    finally:
        MASTER_SEEDS["holdout"] -= cohort*1000003


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenes", type=Path, help="scene_benchmark output with retained request and plan")
    p.add_argument("--run", type=Path, action="append", default=[], help="completed local experiment, repeatable")
    p.add_argument("--gazebo-batch", type=Path, help="closed_loop metadata folders with retained run IDs")
    p.add_argument("--jsonl", type=Path, action="append", default=[], help="raw native benchmark rows; repeatable")
    p.add_argument("--batch", type=Path, help="directory of frozen solve, ablation and timing JSONL files")
    p.add_argument("--cohort", type=int, help="regeneration cohort required for benchmark input hashes")
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if args.batch:
        if args.jsonl:
            p.error("choose --batch or explicit --jsonl inputs")
        args.jsonl = sorted(p for p in args.batch.glob("*.jsonl") if p.name.startswith(("solve_","timing_","ablation_")))
        if not args.jsonl:
            p.error("no native result files in --batch")
    if not (args.scenes or args.run or args.gazebo_batch or args.jsonl) or (args.jsonl and args.cohort is None):
        p.error("select archived inputs; --jsonl requires its frozen --cohort")
    timing = yaml.safe_load(CONTROL.read_text())["timing"]
    rows = list(scene_rows(args.scenes, timing)) if args.scenes else []
    runs = dict.fromkeys(args.run)
    if args.gazebo_batch:
        runs.update({ROOT/json.loads(path.read_text())["archive"]:path.parent for path in sorted(args.gazebo_batch.glob("*/metadata.json"))})
    rows.extend(run_row(path, plant_folder) for path, plant_folder in runs.items())
    rows.extend(bench_rows(args.jsonl, args.cohort or 0, timing))
    groups = {}
    for row in rows:
        group = "/".join([row["task"], row["profile"], row.get("operation","scene" if not row.get("run_id") else "gazebo"),
                          str(row.get("ablation") or "default")])
        summary = groups.setdefault(group, Counter())
        summary["all_requests"] += 1
        summary["policy_compliant_pure_A"] += (row["status"]=="CERTIFIED_POLICY_COMPLIANT" and row.get("metrics", {}).get("pure_A", False))
        if row.get("label")=="A":
            summary["known_A_requests"] += 1
            summary["known_A_policy_compliant_pure_A"] += (row["status"]=="CERTIFIED_POLICY_COMPLIANT" and row.get("metrics", {}).get("pure_A", False))
        if row.get("run_id"):
            summary["closed_loop_passed_with_pure_A_plan"] += (row["status"]=="passed" and row.get("metrics", {}).get("pure_A", False))
    report = dict(model_sample_period_max_s=SAMPLE_S, wheel_order=["front_left","front_right","rear_left","rear_right"],
                  cohort=args.cohort, native_input_files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.jsonl},
                  wheel_units="radians; absolute plan peaks sampled at <=10ms; odometry implies rolling axes, no direct wheel feedback",
                  stop_rule="All active cars below stop_speed inside a normalized same-gear A block, lasting >= max(stable_time, two controller periods); ends excluded",
                  provenance_rule="Unique certified proposal consistent with delivered motion structure; ambiguous and unavailable sources reported as null",
                  steering_control_sha256=hashlib.sha256(CONTROL.read_bytes()).hexdigest(), groups=groups, cases=rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, separators=(",",":"), allow_nan=False)+"\n")
    print(len(rows), "rows", args.output)


if __name__=="__main__":
    main()
