"""Aggregate the matched legacy runs without mixing speed profiles or dropping misses."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np

from cases import LOCAL, ROOT, REPORT

TASKS=("single","formation")
PROFILES=("standard","baseline")


def identity(path):
    h=hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(1<<20),b""):
            h.update(block)
    return h.hexdigest()


def validate_rows(rows, manifest, split, task, profile, timing):
    expected={r["id"]:r for r in manifest["cases"]}
    records=set()
    for row in rows:
        key=row["case_id"]
        declared=expected.get(key,{})
        identity=key,row.get("repeat",0)
        valid=(row["request_hash"]==declared.get("request") and row["map_hash"]==declared.get("map")
               and row["limits_hash"]==declared.get("limits") and row["seed"]==declared.get("seed")
               and row["label"]==declared.get("label") and row["family"]==declared.get("family")
               and row["task"]==task and row["profile"]==profile and key.startswith(f"{split}/{task}/")
               and row["operation"]==("timing" if timing else "solve") and identity not in records)
        if not valid:
            raise ValueError(f"Legacy result does not match frozen {profile} request: {identity}")
        records.add(identity)
    return records


def p95(values):
    return sorted(values)[math.ceil(.95*len(values))-1] if values else None


def report(directory, new_batch=None):
    result={}
    for task in TASKS:
        for profile in PROFILES:
            name=f"{task}_{profile}"; p=directory/f"legacy_{name}.jsonl"
            if not p.exists():continue
            cases=list(map(json.loads,p.read_text().splitlines()))
            manifest=json.loads((REPORT/("case_manifest_baseline.json" if profile=="baseline" else "case_manifest.json")).read_text())
            validate_rows(cases,manifest,"holdout",task,profile,False)
            vals=[r["evidence"] for r in cases]
            feasible=[r for r in cases if r["label"]!="invalid"]
            a_cases=[r for r in cases if r["label"]=="A"]
            summary=dict(attempts=len(cases),unique=len({r["case_id"] for r in cases}),cohorts=sorted({r.get("cohort",0) for r in cases}),
                feasible=len(feasible),certified_in_budget=sum(r["evidence"]["certified_in_budget"] for r in feasible),
                certified_including_late=sum(r["evidence"]["solved"] for r in feasible),
                solved_after_wall_budget=sum(r["evidence"]["solved"] and not r["evidence"]["within_budget"] for r in feasible),
                solved_within_wall_but_over_task_duration=sum(r["evidence"]["solved"] and r["evidence"]["within_budget"] and
                    not r["evidence"]["certified_in_budget"] for r in feasible),
                pure_A_in_budget=sum(r["evidence"]["certified_in_budget"] and r["evidence"].get("modes")==["A"] for r in a_cases),
                known_A=len(a_cases),infrastructure_errors=sum(e["status"]=="ERROR" for e in vals),
                false_executable_invalid=sum(r["evidence"]["solved"] for r in cases if r["label"]=="invalid"),
                status_counts=dict(collections.Counter(e["status"] for e in vals)),raw_path=str(p),raw_sha256=identity(p))
            if task=="single" and any(r["evidence"].get("spin_measurement")!="projected_yaw_change"
                                     for r in a_cases if r["evidence"]["certified_in_budget"]):
                summary["pure_A_in_budget_upper_bound"]=summary.pop("pure_A_in_budget")
                summary["pure_A_note"]="Early legacy projections recorded translating modes only. A run with in-place spin may have been counted as all A."
            if new_batch:
                new_path=new_batch/f"solve_holdout_{profile}.jsonl"
                if new_path.exists():
                    fresh={r["case_id"]:r for r in map(json.loads,new_path.read_text().splitlines()) if r["task"]==task}
                    matched=[]
                    for r in feasible:
                        other=fresh.get(r["case_id"])
                        if other is None:
                            continue
                        if any(other[key]!=r[key] for key in ("request_hash","map_hash","limits_hash")):
                            raise ValueError(f"Pair mismatch at {name}/{r['case_id']}")
                        verified=other["evidence"]
                        ok=verified.get("status")=="CERTIFIED_POLICY_COMPLIANT" and verified.get("audit",{}).get("executable",False)
                        matched.append((r["evidence"]["certified_in_budget"],bool(ok)))
                    summary["paired_feasible"]=dict(total=len(matched),old_only=matched.count((True,False)),
                        new_only=matched.count((False,True)),both_pass=matched.count((True,True)),neither=matched.count((False,False)),
                        new_raw_sha256=identity(new_path),new_source_commits=sorted({r["source_sha"] for r in fresh.values()}))
            f=directory/f"legacy_timing_{name}.jsonl"; timing={}
            if f.exists():
                rows=list(map(json.loads,f.read_text().splitlines()))
                validate_rows(rows,manifest,"core",task,profile,True)
                for family in sorted({r["family"] for r in rows}):
                    subset=[r for r in rows if r["family"]==family]
                    trials=[r for r in subset if r["repeat"]>=0]
                    lat=[r["evidence"]["wall_s"] for r in trials if r["evidence"]["certified_in_budget"]]
                    times=[r["evidence"]["wall_s"] for r in trials if r["evidence"]["wall_s"] is not None]
                    rank=math.ceil(.95*len(trials))
                    policy_p95=sorted(lat)[rank-1] if len(trials)==30 and 1<=rank<=len(lat) else None
                    timing[family]=dict(warmups=sum(r["repeat"]<0 for r in subset),attempts=len(trials),certified_in_budget=len(lat),
                        successful_p50_s=float(np.median(lat)) if lat else None,
                        successful_p95_s=p95(lat),
                        successful_max_s=max(lat,default=None),all_terminal_p95_s=p95(times),
                        policy_compliant_p95_s=policy_p95,
                        policy_compliant_p95_observed=policy_p95 is not None,
                        target_hash=trials[0]["request_hash"] if trials else None,
                        complete={r["repeat"] for r in subset}==set(range(-5,30)),
                        cpus=sorted({cpu for r in subset for cpu in r["cpus"]}))
                if new_batch:
                    peer_path=new_batch/f"timing_core_{profile}.jsonl"
                    if peer_path.exists():
                        peer=[r for r in map(json.loads,peer_path.read_text().splitlines()) if r["task"]==task and r["repeat"]>=0]
                        for family,metrics in timing.items():
                            group=[r for r in peer if r["family"]==family]
                            if not group or not metrics["target_hash"]:
                                continue
                            if any(r["request_hash"]!=metrics["target_hash"] for r in group):
                                raise ValueError(f"Timing request mismatch at {name}/{family}")
                            success=[r["evidence"]["first_policy_compliant_s"] for r in group if
                                     r["evidence"].get("status")=="CERTIFIED_POLICY_COMPLIANT" and
                                     r["evidence"].get("audit",{}).get("executable")]
                            rank=math.ceil(.95*len(group))
                            metrics["new_complete"]=len(group)==30 and {r["repeat"] for r in group}==set(range(30))
                            metrics["new_certified_in_budget"]=len(success)
                            metrics["new_policy_compliant_p95_s"]=sorted(success)[rank-1] if metrics["new_complete"] and rank<=len(success) else None
                            metrics["new_raw_sha256"]=identity(peer_path)
                summary.update(timing=timing,timing_raw_path=str(f),timing_raw_sha256=identity(f))
            result[name]=summary
    code=ROOT/"ros2_ws/install/fwis_backends/lib/fwis_backends"
    base=ROOT/"ros2_ws/install/fwis_backends"
    modules={str(f.relative_to(base)):identity(f) for f in sorted(base.rglob("*"))
             if f.is_file() and (f.suffix in {".py",".so"} or f.name.endswith(".cpython-310-x86_64-linux-gnu.so"))}
    reference="ef51bebb112a2efd79b2d7d8b69f9998f0fdf3d8"
    baseline_unchanged=subprocess.run(["git","diff","--quiet",reference,"--",
        "ros2_ws/src/fwis_backends","ros2_ws/src/fwis_multi_planner_cpp","ros2_ws/src/fwis_planning_core"],
        cwd=ROOT,check=False).returncode==0
    summary=dict(schema="coptrans.v3.previous_backend_comparison.v1",cohorts=result,
        prior_reference=reference,legacy_source_matches_prior_reference=baseline_unchanged,
        comparison_scope="Same frozen request, envelope, speed profile, time budget and task time cap. Previous planner results use its own certification; trials cover implementation-specific ROS transport on single and the in-process API on formation.",
        timing_split="core", timing_quantile="nearest rank, real observations only; deadline failures separately retained",
        archived_development_timing=str(directory/"timing_development"),
        superseded_pre_correction_grid_timing=str(directory/"timing_prior_grid"),
        installed_python_and_native_module_count=len(modules),
        installed_modules_tree_sha256=hashlib.sha256(json.dumps(modules,sort_keys=True).encode()).hexdigest(),
        summary_git_sha=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
        previous_binaries={str(code/f):identity(code/f) for f in ["single_state_lattice_planner.py","single_continuous_mode_path_planner"]},
        runner_sha256=identity(Path(__file__).with_name("legacy_compare.py")))
    return summary


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--directory",type=Path,default=LOCAL.parent/"hybrid_v3_numeric/legacy_c3")
    ap.add_argument("--new-batch",type=Path,default=LOCAL/"release_c6")
    ap.add_argument("--output",type=Path);a=ap.parse_args()
    text=json.dumps(report(a.directory,a.new_batch),indent=2,allow_nan=False)+"\n"
    if a.output:a.output.write_text(text)
    else:print(text,end="")

if __name__=="__main__":main()
