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
sys.path.insert(0,str(ROOT/"ros2_ws/src/fwis_control/test"))
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
        beg,end=times[0],times[-1]
        wall=(end["callback_started_steady_ns"]-beg["callback_started_steady_ns"])*1e-9
        items.append(dict(ticks=len(tracking),stale_motion=stale,ros_misses=sum(sum(deadline_info(t)[1:]) for t in times),
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
    batch=[]
    for file in sorted(folder.glob("*/metadata.json")):
        entry=load(file); archive=ROOT/entry["archive"]
        result=yaml.safe_load((archive/"result.yaml").read_text())
        timed=lines(archive/"timing_trace.jsonl")
        recorded={k:result.get(k) for k in ("plan_received","planner_source","model_sha256","admitted","complete",
                  "start","goal","plan_duration_s","planner_wall_s","routed_plan_keys","tracking_implementation","final_cmd_vel")}
        entry.update(recorded,metrics=result.get("metrics"),fault=load(file.parent/"fault.json"),
                     clocks=clock_metrics(file.parent),admissions=[r for r in timed if r["event"]=="native_group_admission" and r["phase"]=="AUTHORIZED"],
                     result_sha256=sha(archive/"result.yaml"))
        batch.append(entry)
    return batch


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
            row["metrics"]=e.get("metrics",{})
            if raw["operation"]=="witness":row["valid"]=e.get("executable",False)
            records.append(row)
    return records
