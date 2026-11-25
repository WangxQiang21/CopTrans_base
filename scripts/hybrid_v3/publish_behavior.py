#!/usr/bin/env python3
"""Publish section 17.8 aggregates for a complete native batch and Gazebo scene requests."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

from cases import indices, LOCAL, ROOT
from route_evidence import matrix, read_route, HISTORICAL, pair
from evidence_manifest import runtime_binaries


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def identity(row):
    return row["operation"],row["dataset_split"],row["task"],row["profile"],row["case_id"],row["ablation"],row["repeat"]


def check_native(rows):
    expected = set()
    for task in ("single","formation"):
        for profile in ("baseline","standard"):
            for split in ("core","development","holdout","pruning"):
                for idx in range(len(indices(split))):
                    name = f"{split}/{task}/{idx:03}"
                    if split=="pruning":
                        for cond in ("fixed","movable","full"):
                            expected.add(("ablation",split,task,profile,name,cond,0))
                    else:
                        expected.add(("solve",split,task,profile,name,None,0))
                    if split=="core" and idx in (indices(split).index(1),indices(split).index(6)):
                        for rep in range(-5,30):
                            expected.add(("timing",split,task,profile,name,None,rep))
    actual = {identity(row) for row in rows}
    if len(rows)!=len(actual) or actual!=expected:
        raise ValueError(f"Native receipt count {len(rows)}; missing {len(expected-actual)}; unexpected {len(actual-expected)}")
    for row in rows:
        if row["status"]=="CERTIFIED_POLICY_COMPLIANT" and (not row.get("independently_certified") or "metrics" not in row):
            raise ValueError(f"Claimed success lacks validated measurements at {identity(row)}")


def summarize(rows):
    valid = [r for r in rows if r["status"]=="CERTIFIED_POLICY_COMPLIANT" and r["independently_certified"]]
    if any("metrics" not in r for r in valid):
        raise ValueError("a policy-compliant certified model has no metrics")
    ms = [r["metrics"] for r in valid]
    known = [r for r in rows if r.get("label")=="A"]
    count = dict(requests=len(rows), status_counts=Counter(r["status"] for r in rows),
                 policy_compliant_certified=len(valid), pure_A_success=sum(m["pure_A"] for m in ms),
                 known_A_all_requests=len(known), known_A_pure_A_success=sum(
                     r.get("label")=="A" and r["metrics"]["pure_A"] for r in valid),
                 distinct_request_plan_pairs=len({(r.get("request_hash",r.get("request_sha256")),r["plan_sha256"]) for r in valid}))
    if not ms:
        return count
    roles = Counter((b["mode"]+"_"+b["role"]) for m in ms for b in m["auxiliary_roles"])
    a = sum(m["A_length_m"] for m in ms)
    o = sum(m["O_length_m"] for m in ms)
    # Every group contains only single-vehicle or only formation requests.
    cur = np.amax([m["curvature_jump_per_vehicle_m_inv"] for m in ms], axis=0)
    angles = np.amax([m["plan_peak_abs_wheel_angle_rad"] for m in ms], axis=0)
    return dict(count, normalized_sequences=Counter(m["normalized_modes"] for m in ms),
                A_translation_fraction=a/(a+o) if a+o>0 else None, A_length_sum_m=a, O_length_sum_m=o,
                S_abs_angle_sum_all_vehicles_rad=sum(m["S_angle_rad"] for m in ms),
                S_block_sum=sum(m["S_blocks"] for m in ms), auxiliary_block_sum=sum(m["aux_blocks"] for m in ms),
                auxiliary_blocks_by_role=roles, longest_continuous_A_max_m=max(m["longest_continuous_A_m"] for m in ms),
                internal_stop_events=sum(m["internal_stops"] for m in ms),
                internal_stop_events_by_vehicle=np.sum([m["stops_by_vehicle"] for m in ms],axis=0).tolist(),
                max_join_curvature_jump_by_vehicle_m_inv=cur.tolist(), max_abs_wheel_angles_by_vehicle_rad=angles.tolist(),
                net_seed_to_final_aux_reduction_histogram=Counter(str(m["net_aux_block_reduction"]) for m in ms),
                structural_attempt_totals=sum((Counter(m["structural_edits_attempted"] or {}) for m in ms),Counter()),
                winning_proposal_counts=Counter(m["winning_proposal"] or "ambiguous_or_unavailable" for m in ms))


def release(native_path, scene_dir):
    native = read(native_path)
    scenes = read(scene_dir/"behavior.json")
    receipt = read(scene_dir/"summary.json")
    rows = native["cases"]
    check_native(rows)
    if scenes["steering_control_sha256"]!=native["steering_control_sha256"]:
        raise ValueError("different controller timing configurations")
    ids = {(r["source_sha"],r["exe_hash"],r["lib_hash"],r["ranking_policy_sha256"]) for r in rows}
    if len(ids)!=1:
        raise ValueError("mixed native solver identities")
    system, exe, lib, policy = ids.pop()
    if (exe,lib,policy)!=(receipt["binary_sha256"],receipt["library_sha256"],receipt["policy_sha256"]):
        raise ValueError("scene requests used another solver or policy")
    scene_meta = {(r["family"],r["task"],r["profile"]):r for r in receipt["cases"]}
    scene_ids = {(r["family"],r["task"],r["profile"]) for r in scenes["cases"]}
    if len(scene_ids)!=len(scenes["cases"]) or scene_ids!=scene_meta.keys():
        raise ValueError("scene matrix differs from its archived receipts")
    grouped = defaultdict(list)
    for row in rows:
        condition = row["ablation"] or "default"
        if row["operation"]=="timing":
            condition = "warmup" if row["repeat"]<0 else "formal"
        grouped["/".join((row["operation"], row["dataset_split"], row["task"], row["profile"], condition))].append(row)
    named = {}
    for row in scenes["cases"]:
        key = row["family"],row["task"],row["profile"]
        archived = scene_meta[key]
        if row["request_sha256"]!=archived["request_sha256"]:
            raise ValueError(f"scene request mismatch: {key}")
        grouped["/".join(("scene",row["task"],row["profile"]))].append(row)
        m = row.get("metrics", {})
        named["/".join(key)] = dict(status=row["status"], request_sha256=row["request_sha256"], plan_sha256=row.get("plan_sha256"),
                                    sequence=m.get("normalized_modes"), A_fraction=m.get("A_translation_fraction"),
                                    stop_events=m.get("internal_stops"), longest_A_m=m.get("longest_continuous_A_m"),
                                    wheel_peak_rad=max(map(max,m["plan_peak_abs_wheel_angle_rad"])) if m else None,
                                    auxiliary_roles=[b["mode"]+"_"+b["role"] for b in m.get("auxiliary_roles",[])])
    holdout = [r for r in rows if r["operation"]=="solve" and r["dataset_split"]=="holdout"]
    known = [r for r in holdout if r["label"]=="A"]
    def passed(r):
        return r["status"]=="CERTIFIED_POLICY_COMPLIANT" and r["independently_certified"]
    return dict(schema="coptrans_hybrid_v3_behavior/1", source_commit=system, core_source_commit=receipt["core_source_commit"],
                exe_sha256=exe, library_sha256=lib, ranking_policy_sha256=policy, cohort=native["cohort"],
                steering_control_sha256=native["steering_control_sha256"], source_jsonl_sha256=native["native_input_files"],
                archive_hashes={"native_behavior":sha(native_path),"scene_receipts":sha(scene_dir/"summary.json"),
                                "scene_behavior":sha(scene_dir/"behavior.json")},
                measurement=dict(source="native certified planning model",max_plan_sample_period_s=native["model_sample_period_max_s"],
                                 wheel_order=native["wheel_order"],stop_rule=native["stop_rule"],
                                 frozen_stop_contracts=[dict(stop_speed_mps=speed,min_duration_s=window) for speed,window in
                                     sorted({(r["metrics"]["stop_speed_mps"],r["metrics"]["stop_window_s"]) for r in rows if r.get("metrics")})],
                                 auxiliary_roles="initial, interior, terminal, or sole normalized motion block",
                                 timing="5 warmups and 30 formal repeats kept per task, speed and representative family",
                                 lengths="reference-frame arc length, distance-weighted A/(A+O); sum of absolute S angles across all vehicles",
                                 aggregates="motion metrics use policy-compliant, independently certified plans; failures remain in the requests denominator",
                                 provenance=native["provenance_rule"]),
                coverage=dict(native_rows=len(rows),native_by_operation=Counter(r["operation"] for r in rows),
                              native_by_split=Counter(r["dataset_split"] for r in rows), known_A_holdout_requests=len(known),
                              known_A_holdout_pure_A_success=sum(passed(r) and r["metrics"]["pure_A"] for r in known),
                              native_internal_stop_events=sum(r["metrics"]["internal_stops"] for r in rows if passed(r)),
                              scene_rows=len(scenes["cases"]),scene_passed=sum(r["passed"] and passed(r) for r in scenes["cases"]),
                              scene_internal_stop_events=sum(r["metrics"]["internal_stops"] for r in scenes["cases"] if passed(r))),
                groups={key:summarize(grouped[key]) for key in sorted(grouped)}, scene_families=named)


def readme_paragraph(report):
    c = report["coverage"]
    text = (f"按冻结轨迹重算了 {c['native_rows']} 条原生实验及 {c['scene_rows']} 个场景请求。"
            f"已知纯 Ackermann 可行的留出任务中，{c['known_A_holdout_pure_A_success']}/{c['known_A_holdout_requests']} 个请求"
            f"返回通过独立认证的纯 Ackermann 计划。原生各组共检出 {c['native_internal_stop_events']} 次 A 段内部停车。")
    if "gazebo_gui" in report:
        gui,headless = report["gazebo_gui"],report["gazebo_headless"]
        text += (f"已核对 {gui['runs']} 轮双 GUI 与 {headless['runs']} 轮无 GUI 实验，"
                 f"按各自原生时钟记录分别发现 {gui['ros_time_misses']} 和 {headless['ros_time_misses']} 次仿真时钟误期。")
    return text+"逐组的辅助段位置、曲率跳变和各轮转角峰值见 [behavior.json](behavior.json)。"


def add_gazebo(report, path, clocks, system, *, gui=True):
    rows = read(path)["cases"]
    if not matrix(rows,gui):
        raise ValueError("Gazebo publication requires every named family and exact trial once, with all failures retained")
    system_sha=(system/"source_commit").read_text().strip()
    binaries=runtime_binaries(system)
    libs={h for name,h in binaries.items() if Path(name).name=="libfwis_hybrid_joint.so"}
    if len(binaries)!=5 or libs!={report["library_sha256"]}:
        raise ValueError("The system snapshot must contain the frozen native solver")
    campaign, = {r["qualification_group"] for r in rows}
    refs={name:read_route(ROOT/"reports/runs"/i) for name,i in HISTORICAL.items()}
    for row in rows:
        meta=read(LOCAL/campaign/row["run_id"]/"metadata.json")
        if meta["source_commit"]!=system_sha or meta["installed_binaries"]!=binaries or row["gui"]!=gui:
            raise ValueError("Gazebo experiment belongs to another system snapshot: "+row["run_id"])
        route=read_route(ROOT/meta["archive"])
        if (not route["exact_archive"] or route["status"]!=row["status"] or route["run_id"]!=row["run_id"] or
                route["model_sha256"]!=row.get("plan_sha256")):
            raise ValueError("Gazebo experiment archive mismatch: "+row["run_id"])
        row["route"]=route
        if row["family"] in HISTORICAL:
            compared=pair({**row,"historical":meta.get("historical")},refs[row["family"]])
            if not compared["route_matches_archive"]:
                raise ValueError("Gazebo historical request differs: "+row["run_id"])
            row["historical_pair"]=compared
    audited = {r["id"]:r for k,r in clocks.items() if k.startswith(campaign+"/")}
    if audited.keys()!={r["run_id"] for r in rows}:
        raise ValueError(f"Native clock review and {campaign} have different experiment IDs")
    for run in rows:
        if any(run[k]!=audited[run["run_id"]][k] for k in ("status","gui","family","trial","profile","task")):
            raise ValueError(f"Native clock review differs at {run['run_id']}")
    grouped = defaultdict(list)
    for row in rows:
        grouped["/".join((row["task"],row["profile"],row["family"]))].append(row)
    outputs = {}
    for key, group in sorted(grouped.items()):
        def extrema(values, op):
            valid = [v for v in values if v is not None]
            return op(valid) if valid else None
        def wheelmax(values):
            valid = [v for v in values if v and all(i is not None for i in v)]
            return np.amax(valid, axis=0).tolist() if valid else None
        poses = [r["measured_geometry"] for r in group]
        odoms = [r["wheel_odometry"] for r in group if r.get("wheel_odometry")]
        models = [r["metrics"] for r in group if r.get("metrics")]
        outputs[key] = dict(runs=len(group),run_ids=[r["run_id"] for r in group],status_counts=Counter(r["status"] for r in group),
            request_chord_m=[r["route"]["chord_m"] for r in group],measured_arc_m=[r["route"]["motion"]["arc_m"] for r in group],
            sampled_mode_switches_by_vehicle=[r["route"]["motion"]["sampled_feedback_switches"] for r in group],
            historical_pairs=[r["historical_pair"] for r in group if "historical_pair" in r],
            delivered_authorities=len(models), normalized_plan_modes=Counter(m["normalized_modes"] for m in models),
            plan_wheel_peak_rad=wheelmax(m["plan_peak_abs_wheel_angle_rad"] for m in models),
            odom_implied_wheel_peak_rad=wheelmax(o["odom_implied_peak_abs_wheel_angle_rad"] for o in odoms),
            applied_command_implied_wheel_peak_rad=wheelmax([
                w["implied_peak_abs_wheel_angle_rad"] if w else None for w in o["applied_twist_by_vehicle"]] for o in odoms),
            odometry_runs=len(odoms),max_odom_sample_gap_s=extrema((o["max_odom_sample_gap_s"] for o in odoms),max),
            max_body_terminal_error_m=extrema((v for g in poses for v in g["terminal_position_error_m_by_vehicle"].values()),max),
            max_body_terminal_yaw_error_deg=extrema((v for g in poses for v in g["terminal_yaw_error_deg_by_vehicle"].values()),max),
            max_reference_terminal_error_m=extrema((g["terminal_reference_position_error_m"] for g in poses),max),
            max_reference_terminal_yaw_error_deg=extrema((g["terminal_reference_yaw_error_deg"] for g in poses),max),
            max_mount_error_m=extrema((g["max_mount_error_m"] for g in poses),max),
            min_body_gap_m=extrema((g["min_body_gap_m"] for g in poses),min),
            min_environment_clearance_m=extrema((v for g in poses for v in g["min_environment_clearance_m"].values()),min))
    group = "gazebo_gui" if gui else "gazebo_headless"
    report[group] = dict(raw_behavior_sha256=sha(path), runs=len(rows), required_unique_trials=96 if gui else 4,system_commit=system_sha,library_sha256=report["library_sha256"],
                         campaign=campaign, clock_row_sha256=hashlib.sha256(json.dumps(audited,sort_keys=True,separators=(",",":")).encode()).hexdigest(),
                         ros_time_misses=sum(r["ros_misses"] for r in audited.values()),
                         stale_motion=sum(r["stale_motion"] for r in audited.values()),
                         independent_clock_flags=Counter(flag for r in audited.values() for flag in r["flags"]),
                         wheel_angle_sensor_samples=0,groups=outputs)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--native",type=Path,required=True, help="raw behavior_metrics --batch output")
    p.add_argument("--scenes",type=Path,required=True, help="scene receipts and recomputed behavior.json")
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--gazebo",type=Path,help="behavior_metrics --gazebo-batch output after all GUI matrix trials")
    p.add_argument("--headless",type=Path,help="behavior_metrics output for the separate four headless trials")
    p.add_argument("--system",type=Path,default=LOCAL/"system_c7_long",help="clean system snapshot used for all Gazebo trials")
    p.add_argument("--clock-receipts",type=Path,help="independent native-clock receipt index for both simulation scopes")
    a=p.parse_args()
    report=release(a.native,a.scenes)
    if a.gazebo or a.headless:
        if not (a.gazebo and a.headless and a.clock_receipts):
            p.error("final simulation publication requires --gazebo, --headless and --clock-receipts")
        clocks = read(a.clock_receipts)
        add_gazebo(report,a.gazebo,clocks,a.system,gui=True)
        add_gazebo(report,a.headless,clocks,a.system,gui=False)
    a.output.write_text(json.dumps(report,separators=(",",":"),sort_keys=True,allow_nan=False)+"\n")
    print(readme_paragraph(report))


if __name__=="__main__":
    main()
