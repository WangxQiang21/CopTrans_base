"""Reduce raw per-attempt evidence; keep experiments and full coefficients in local storage."""
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml
from cases import LOCAL, REPORT, ROOT, MASTER_SEEDS, POLICY, GEOMETRY, REGISTRY, SPEC, digest
sys.path.append(str(ROOT/"ros2_ws/src/fwis_control/test"))
from analyze_control_timing import deadline_info, pair_commands, plant_gaps

NUMERIC=LOCAL.parent/"hybrid_v3_numeric"
TASKS=("single","formation")
PROFILES=("standard","baseline")


def load(path):
    if not path.is_file() or not path.stat().st_size:return None
    return json.loads(path.read_text())


def quant(values,q):
    if not values:return None
    ordered=sorted(values)
    return ordered[max(0,math.ceil(q*len(ordered))-1)]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def lines(path):
    if not path.exists():return []
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def xml_cases(folder):
    result={}
    for p in folder.glob("*.xml"):
        for case in ET.parse(p).iter("testcase"):
            name=case.attrib["name"]
            result[name]=result.get(name,True) and not any(case.find(x) is not None for x in ("failure","error","skipped"))
    return result


def setup_metrics(folder):
    groups={}
    for cars in (1,3):
        summary=load(folder/f"{cars}.summary.json")
        if not summary:continue
        with (folder/f"{cars}.csv").open() as inp:
            rows=list(csv.DictReader(inp))
        for index in range(1,cars+1):
            batch=[{k:int(v) for k,v in r.items()} for r in rows if int(r["vehicle"])==index]
            if not batch:continue
            per=40000000
            compute=[r["tracking_compute_ns"]*1e-6 for r in batch]
            jitter=[r["release_jitter_ns"]*1e-6 for r in batch]
            groups[f"{cars}/{index}"]={**summary,"ticks":len(batch),"valid":sum(r["valid"]==1 for r in batch),
                "consecutive":all(r["tick"]==k and r["scheduled_ns"]==k*per for k,r in enumerate(batch)),
                "deadline_misses":sum(r["command_submitted_ns"]>r["scheduled_ns"]+per for r in batch),
                "p99_compute_ms":quant(compute,.99),"max_compute_ms":max(compute),
                "max_callback_ms":max((r["command_submitted_ns"]-r["scheduled_ns"]-r["release_jitter_ns"])*1e-6 for r in batch),
                "p99_release_jitter_ms":quant(jitter,.99),"max_release_jitter_ms":max(jitter),
                "csv_sha256":sha(folder/f"{cars}.csv")}
    return groups


def native_stress_comparison(folder, benchmark):
    """Keep the original overlapping-load observations alongside the isolated native gate."""
    expected = sha(benchmark)
    files = {}
    misses = []
    for cars in (1, 3):
        for suffix in ("csv", "summary.json", "binary.sha256", "libraries.txt"):
            path = folder / f"{cars}.{suffix}"
            files[str(path.resolve())] = sha(path)
        identity = (folder / f"{cars}.binary.sha256").read_text().split()[0]
        if identity != expected:
            raise ValueError(f"{cars}-vehicle stress run used a different native load binary")
        with (folder / f"{cars}.csv").open() as src:
            for row in csv.DictReader(src):
                delay = int(row["command_submitted_ns"]) - int(row["scheduled_ns"])
                if delay > 40_000_000:
                    jitter = int(row["release_jitter_ns"])
                    misses.append({
                        "cars": cars, "vehicle": int(row["vehicle"]), "tick": int(row["tick"]),
                        "release_jitter_ns": jitter, "tracking_compute_ns": int(row["tracking_compute_ns"]),
                        "release_to_submit_ns": delay - jitter, "deadline_excess_ns": delay - 40_000_000,
                    })
    metrics = setup_metrics(folder)
    if set(metrics) != {"1/1", "3/1", "3/2", "3/3"} or sum(r["deadline_misses"] for r in metrics.values()) != len(misses):
        raise ValueError("Incomplete stress measurement")
    return {"raw_sha256": files, "binary_sha256": expected, "setups": metrics, "misses": misses}


def clock_metrics(folder):
    items=[]
    for f in sorted(folder.glob("robot*_controller.jsonl.gz")):
        with gzip.open(f,"rt") as h:
            rows=[json.loads(x) for x in h]
        tracking=[r for r in rows if r.get("phase")=="TRACKING" and r.get("control_timing")]
        if not tracking:continue
        times=[r["control_timing"] for r in tracking]
        stale=sum(any(r["command"]) and (r["stamp"]-r["control_timing"]["odom_stamp_ns"]*1e-9>.20 or
                   r["control_timing"]["callback_started_steady_ns"]-r["control_timing"]["odom_callback_steady_ns"]>2e8)
                  for r in tracking)
        plant=f.with_name(f.name.split("_")[0]+"_plant_timing.csv.gz")
        if not plant.exists():continue
        with gzip.open(plant,"rt") as h:prows=list(csv.DictReader(h))
        # Use only uniquely attributable commands to measure DDS and plant delivery.
        applied=[r for r in prows if r.get("event")=="command"]
        all_timed=[r for r in rows if r.get("control_timing") and "command" in r]
        pairs,unmatched=pair_commands(all_timed,applied)
        tracked={r["control_timing"]["command_publish_steady_ns"] for r in tracking}
        pairs=[(start,row) for start,row in pairs if start in tracked]
        odom_gaps,physics=plant_gaps(prows)
        consumed=[(int(row["consume_steady_ns"])-start)*1e-6 for start,row in pairs]
        callback_ms=[(t["callback_ended_steady_ns"]-t["callback_started_steady_ns"])*1e-6 for t in times]
        publish_ms=[(t["command_published_steady_ns"]-t["command_publish_steady_ns"])*1e-6 for t in times]
        begin_ns=times[0]["callback_started_steady_ns"];end_ns=times[-1]["callback_started_steady_ns"]
        gaps=[r for r in physics if begin_ns<=r["begin_steady_ns"]<=end_ns]
        wall_gaps=[r for r in odom_gaps if begin_ns<=r["begin_steady_ns"]<=end_ns]
        inter_rtf=[(b["timer_called_clock_ns"]-a["timer_called_clock_ns"])/(b["callback_started_steady_ns"]-a["callback_started_steady_ns"])
                   for a,b in zip(times,times[1:]) if (b["callback_started_steady_ns"]-a["callback_started_steady_ns"])>=40000000]
        # An injected stop can allow exactly one TRACKING tick. Bracket that
        # measured tick with its adjacent scheduled callbacks for the RTF only.
        beg,end=times[0],times[-1]
        rtf_window="tracking"
        if len(times)==1:
            where=next(i for i,row in enumerate(all_timed) if row["control_timing"] is beg)
            beg=all_timed[max(0,where-1)]["control_timing"]
            end=all_timed[min(len(all_timed)-1,where+1)]["control_timing"]
            rtf_window="adjacent_callbacks"
        wall=(end["callback_started_steady_ns"]-beg["callback_started_steady_ns"])*1e-9
        items.append(dict(vehicle=f.name.split("_")[0],rtf_window=rtf_window,ticks=len(tracking),stale_motion=stale,ros_misses=sum(sum(deadline_info(t)[1:]) for t in times),
                          wall_s=wall,real_time_factor=((end["timer_called_clock_ns"]-beg["timer_called_clock_ns"])*1e-9/wall if wall>0 else None),
                          max_callback_ms=max(callback_ms),p99_callback_ms=quant(callback_ms,.99),
                          max_publish_ms=max(publish_ms),p99_publish_ms=quant(publish_ms,.99),
                          min_intertick_rtf=min(inter_rtf,default=None),observed_physics_gaps=len(gaps),observed_odom_gaps=len(wall_gaps),
                          max_physics_gap_ms=max((r["wall_ns"]*1e-6 for r in gaps),default=None),
                          max_wall_gap_ms=max([0]+[(b["callback_started_steady_ns"]-a["callback_started_steady_ns"])*1e-6 for a,b in zip(times,times[1:])]),
                          matched_commands=len(pairs),unmatched_commands=dict(unmatched),max_consumption_ms=max(consumed,default=None),
                          p95_consumption_ms=quant(consumed,.95),p99_consumption_ms=quant(consumed,.99),
                          max_odom_dds_ms=max((t["odom_received_realtime_ns"]-t["odom_source_realtime_ns"])*1e-6 for t in times)))
    return items


def closed_rows(folder):
    from route_evidence import read_route
    batch=[]
    for file in sorted(folder.glob("*/metadata.json")):
        entry=load(file); archive=ROOT/entry["archive"]
        result=yaml.safe_load((archive/"result.yaml").read_text())
        timed=lines(archive/"timing_trace.jsonl")
        recorded={k:result.get(k) for k in ("plan_received","planner_source","model_sha256","admitted","complete",
                  "start","goal","plan_duration_s","planner_wall_s","routed_plan_keys","tracking_implementation","final_cmd_vel")}
        request=next((e for e in timed if e["event"]=="request_publish"),{})
        received=next((e for e in timed if e["event"]=="plan_received"),{})
        public=next((e for e in timed if e["event"]=="planner_publication_status_received" and
                     e.get("plan_id")==received.get("plan_id")),{})
        manifest=load(archive/"experiment.json") or {}
        model_file=archive/(manifest.get("model_artifact") or "trajectories/model.json.gz")
        model=None
        if model_file.is_file():
            with gzip.open(model_file,"rb") as inp:
                encoded=inp.read()
            if hashlib.sha256(encoded).hexdigest()!=manifest.get("model_sha256"):
                raise ValueError("Archived authority digest mismatch for "+archive.name)
            model=json.loads(encoded)
        publish=(model or {}).get("publication_steady_ns")
        behavior={}
        if model and model.get("phases"):
            from behavior_metrics import plan_metrics
            timing=yaml.safe_load((archive/"controller_parameters.yaml").read_text())["timing"]
            behavior=plan_metrics(model["request"],model["phases"],timing)
        modes=list(behavior.get("normalized_modes",""))
        entry["plan_behavior"]={k:behavior[k] for k in (
            "pure_A","A_translation_fraction","A_length_m","O_length_m","S_angle_rad","longest_continuous_A_m",
            "internal_stops","curvature_jump_per_vehicle_m_inv","plan_peak_abs_wheel_angle_rad") if k in behavior}
        authority_req=(model or {}).get("request")
        entry["authority_request_sha256"]=digest(authority_req) if authority_req else None
        entry["motion_modes"]=modes
        entry["planned_mode_switches"]=len(modes)-1 if modes else None
        entry.update(recorded,publish_s=(publish*1e-9-request["wall_s"] if publish and "wall_s" in request else None),
                     publication_to_receive_s=(received["wall_s"]-publish*1e-9 if publish and "wall_s" in received else None),
                     map_sha256=manifest.get("map_sha256"),scene_sha256=manifest.get("scene_sha256"),
                     config_sha256=sha(archive/"resolved_config.yaml"),source_sha=manifest.get("git_commit"),
                     request_to_receive_s=(received["wall_s"]-request["wall_s"] if "wall_s" in request and "wall_s" in received else None),
                     publication_event=public,metrics=result.get("metrics"),fault=load(file.parent/"fault.json"),
                     clocks=clock_metrics(file.parent),admissions=[r for r in timed if r["event"]=="native_group_admission" and r["phase"]=="AUTHORIZED"],
                     result_sha256=sha(archive/"result.yaml"),route=read_route(archive),campaign=folder.name,
                     evidence_files={p.name:sha(p) for p in sorted(file.parent.iterdir()) if p.is_file() and
                                     (p.name in ("metadata.json","fault.json") or p.name.endswith((".csv",".csv.gz",".jsonl.gz")))})
        batch.append(entry)
    return batch



def archived_comparisons(closed,default_runs):
    """Pair reference experiments by physical task, map, speed and geometry."""
    reference_ids=("20261125_140950_e40cIk","20261125_141134_AeDvqX","20261125_141201_TcTcHJ")
    def read(run_id):
        folder=ROOT/"reports/runs"/run_id
        if not (folder/"result.yaml").is_file():
            return None
        manifest=load(folder/"experiment.json")
        result=yaml.safe_load((folder/"result.yaml").read_text())
        cfg=yaml.safe_load((folder/"resolved_config.yaml").read_text())
        if result.get("status")!="passed":
            return None
        signature=[cfg["scene_task"],cfg["speed_profile"],manifest["scene_sha256"],manifest["map_sha256"],
                   cfg["start"],cfg["goal"],result.get("formation_offsets_m"),cfg.get("controller_limit_overrides"),
                   cfg.get("plant_overrides")]
        vs=result["metrics"]["vehicles"].values()
        return dict(run_id=run_id,signature=signature,backend=cfg["planner_backend"],result_sha256=sha(folder/"result.yaml"),
                    planning_s=result.get("planner_wall_s"),duration_s=result.get("plan_duration_s"),
                    terminal_m=max(v["terminal_position_error_m"] for v in vs),
                    worst_car_rmse_m=max(v["position_rmse_m"] for v in vs),mount_m=result["metrics"].get("max_mount_error_m"))
    ids=[r["run_id"] for r in closed if r["status"]=="passed"]+[r["run_id"] for r in default_runs or []]
    candidates=[r for run_id in ids if (r:=read(run_id)) is not None]
    result=[]
    for run_id in reference_ids:
        legacy=read(run_id)
        if legacy is not None:
            result.append(dict(reference=legacy,matches=[r for r in candidates if r["signature"]==legacy["signature"]]))
    return result


def collect(batch):
    records=[]
    for f in sorted(batch.glob("*.jsonl")):
        for raw in lines(f):
            e=raw.pop("evidence")
            row={**raw,"status":e.get("status"),"reason":e.get("reason"),"result_hash":digest(e)}
            row.update({k:e.get(k) for k in ("wall_s","actual_backend","first_certified_s","first_policy_compliant_s",
                       "duration_s","deadline_overrun_s","stage_times")})
            row["valid"]=e.get("audit",{}).get("executable",False)
            row["has_plan"]=bool(e.get("plan"))
            row["trace"]=e.get("trace",[])
            row["certificate"]=e.get("audit")
            proposal=next(iter((e.get("stage_times") or {}).get("attempts",[])),{}).get("proposal","")
            row["mode_sequence_before"]=("ASOSA" if row.get("ablation") else proposal.split(":")[1] if ":" in proposal else None)
            row["metrics"]=e.get("metrics",{})
            if raw["operation"]=="witness":row["valid"]=e.get("executable",False)
            records.append(row)
    return records
