"""Aggregate the matched legacy runs without mixing speed profiles or dropping misses."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from cases import LOCAL, ROOT

TASKS=("single","formation")
PROFILES=("standard","baseline")


def identity(path):
    h=hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(1<<20),b""):
            h.update(block)
    return h.hexdigest()


def report(directory):
    result={}
    for task in TASKS:
        for profile in PROFILES:
            name=f"{task}_{profile}"; p=directory/f"legacy_{name}.jsonl"
            if not p.exists():continue
            cases=list(map(json.loads,p.read_text().splitlines()))
            vals=[r["evidence"] for r in cases]
            feasible=[r for r in cases if r["label"]!="invalid"]
            a_cases=[r for r in cases if r["label"]=="A"]
            summary=dict(attempts=len(cases),unique=len({r["case_id"] for r in cases}),cohorts=sorted({r.get("cohort",0) for r in cases}),
                feasible=len(feasible),certified_in_budget=sum(r["evidence"]["certified_in_budget"] for r in feasible),
                certified_including_late=sum(r["evidence"]["solved"] for r in feasible),
                pure_A_in_budget=sum(r["evidence"]["certified_in_budget"] and r["evidence"].get("modes")==["A"] for r in a_cases),
                known_A=len(a_cases),infrastructure_errors=sum(e["status"]=="ERROR" for e in vals),
                false_executable_invalid=sum(r["evidence"]["solved"] for r in cases if r["label"]=="invalid"),
                status_counts=dict(collections.Counter(e["status"] for e in vals)),raw_path=str(p),raw_sha256=identity(p))
            f=directory/f"legacy_timing_{name}.jsonl"; timing={}
            if f.exists():
                rows=list(map(json.loads,f.read_text().splitlines()))
                for family in sorted({r["family"] for r in rows}):
                    subset=[r for r in rows if r["family"]==family]
                    trials=[r for r in subset if r["repeat"]>=0]
                    lat=[r["evidence"]["wall_s"] for r in trials if r["evidence"]["certified_in_budget"]]
                    times=[r["evidence"]["wall_s"] for r in trials if r["evidence"]["wall_s"] is not None]
                    timing[family]=dict(warmups=sum(r["repeat"]<0 for r in subset),attempts=len(trials),certified_in_budget=len(lat),
                        successful_p50_s=float(np.median(lat)) if lat else None,
                        successful_p95_s=float(np.percentile(lat,95)) if lat else None,
                        successful_max_s=max(lat,default=None),all_terminal_p95_s=float(np.percentile(times,95)) if times else None,
                        cpus=sorted({cpu for r in subset for cpu in r["cpus"]}))
                summary.update(timing=timing,timing_raw_path=str(f),timing_raw_sha256=identity(f))
            result[name]=summary
    code=ROOT/"ros2_ws/install/fwis_backends/lib/fwis_backends"
    summary=dict(schema="coptrans.v3.previous_backend_comparison.v1",cohorts=result,
        source_sha=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),
        previous_binaries={str(code/f):identity(code/f) for f in ["single_state_lattice_planner.py","single_continuous_mode_path_planner"]},
        runner_sha256=identity(Path(__file__).with_name("legacy_compare.py")))
    return summary


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--directory",type=Path,default=LOCAL.parent/"hybrid_v3_numeric")
    ap.add_argument("--output",type=Path);a=ap.parse_args()
    text=json.dumps(report(a.directory),indent=2)+"\n"
    if a.output:a.output.write_text(text)
    else:print(text,end="")

if __name__=="__main__":main()
