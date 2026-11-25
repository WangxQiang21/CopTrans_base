#!/usr/bin/env python3
"""Construct oracles separately, freeze inputs, then exercise the native solver blind."""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from cases import LOCAL, MASTER_SEEDS, POLICY, REPORT, ROOT, digest, indices, make, packed


def identify(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def invoke(exe, message, timeout):
    started=time.monotonic()
    try:
        proc=subprocess.run([str(exe)],input=packed(message)+b"\n",stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,timeout=timeout,check=False,
                            env=dict(os.environ,OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1"))
        if proc.returncode:
            return dict(status="process_failed",reason=proc.stderr.decode(errors="replace")[-1500:],
                        exit_code=proc.returncode)
        return json.loads(proc.stdout.splitlines()[-1])
    except subprocess.TimeoutExpired:
        return dict(status="watchdog_timeout",wall_s=time.monotonic()-started)
    except (IndexError,ValueError) as exc:
        return dict(status="malformed_process_output",reason=str(exc))


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument("action",choices=["manifest","witness","solve","timing","ablation"])
    a.add_argument("--split",choices=list(MASTER_SEEDS),default="development")
    a.add_argument("--task",choices=["both","single","formation"],default="both")
    a.add_argument("--profile",choices=["standard","baseline"],default="standard")
    a.add_argument("--ids",help="zero-based comma-separated case indices")
    a.add_argument("--jobs",type=int,default=1)
    a.add_argument("--cohort",type=int,default=0,help="new independent sealed cohort after retuning")
    a.add_argument("--executable",type=Path,default=ROOT/"ros2_ws/install/fwis_hybrid_joint/lib/fwis_hybrid_joint/hybrid_joint_case")
    a.add_argument("--output",type=Path)
    a.add_argument("--source-commit",help="commit of the immutable native build used for this run")
    a.add_argument("--policy-source",type=Path,default=ROOT/"ros2_ws/src/fwis_hybrid_joint/config/policy.json",
                   help="exact policy used to compile the supplied solver")
    args=a.parse_args()
    if args.cohort<0 or args.jobs<1 or (args.action=="timing" and args.jobs!=1):
        a.error("invalid cohort or parallel timing run")
    MASTER_SEEDS["holdout"]+=args.cohort*1000003
    tasks=["single","formation"] if args.task=="both" else [args.task]
    exe=args.executable.resolve()
    output=args.output or LOCAL/f"{args.action}_{args.split}_{args.task}_{args.profile}_c{args.cohort}.jsonl"
    output.parent.mkdir(parents=True,exist_ok=True)
    solver_sha=identify(exe) if args.action!="manifest" else None
    lib_sha=None
    if solver_sha:
        imported=subprocess.check_output(["ldd",str(exe)],text=True)
        lib=next(line.split("=>",1)[1].split(" (")[0].strip() for line in imported.splitlines()
                 if "libfwis_hybrid_joint.so =>" in line)
        lib_sha=identify(Path(lib))
    if args.action=="manifest":
        manifest={"generator_sha256":identify(Path(__file__).with_name("cases.py")),"cohort":args.cohort,
                  "master_seeds":MASTER_SEEDS,"policy":POLICY,"cases":[]}
        for split in MASTER_SEEDS:
            for task in tasks:
                for i in range(len(indices(split))):
                    r,w,meta=make(split,i,task,args.profile)
                    manifest["cases"].append(dict(id=f"{split}/{task}/{i:03}",**meta,request=digest(r),
                                                  map=digest(r["grid"]),limits=digest(r["limits"]),
                                                  witness=digest(w) if w else None))
        document=packed(manifest)+b"\n"
        target=args.output or REPORT/"case_manifest.json"
        if target.exists() and target.read_bytes()!=document:
            raise SystemExit("Manifest already frozen with different content. Use a fresh --cohort and --output.")
        target.write_bytes(document)
        print(f"{len(manifest['cases'])} cases; SHA-256 {identify(target)}")
        return
    # Rechecking a supplied reference trajectory does not run the ranking policy.
    ranking_sha=None if args.action=="witness" else identify(args.policy_source)
    splits=indices(args.split)
    ids=[int(x) for x in args.ids.split(",")] if args.ids else list(range(len(splits)))
    if args.action=="timing" and not args.ids:
        ids=[splits.index(1),splits.index(6)]
    if output.exists():
        raise SystemExit(f"Evidence already exists: {output}. Set --output to a new attempt.")
    harness_commit=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
    sources=subprocess.check_output(["git","rev-parse","--verify",(args.source_commit or "HEAD")+"^{commit}"],cwd=ROOT,text=True).strip()
    if args.source_commit:
        source_tree=subprocess.check_output(["git","rev-parse",sources+":ros2_ws/src"],cwd=ROOT,text=True).strip()
    else:
        source_tree=subprocess.check_output([sys.executable,str(ROOT/"scripts/source_fingerprint.py"),str(ROOT)],text=True).strip()
    def trial(key):
        task,i,rep,ablation=key
        r,w,meta=make(args.split,i,task,args.profile)
        budget=POLICY["budget_"+task]
        config_hash=digest(dict(policy=POLICY,ranking_sha256=ranking_sha,limits_hash=digest(r["limits"]),task=task,ablation=ablation))
        row=dict(task=task,index=i,case_id=f"{args.split}/{task}/{i:03}",dataset_split=args.split,**meta,
                 profile=args.profile,ranking_policy_sha256=ranking_sha,config_hash=config_hash,map_hash=digest(r["grid"]),limits_hash=digest(r["limits"]),
                 request_hash=digest(r),source_sha=sources,source_tree=source_tree,harness_commit=harness_commit,exe_hash=solver_sha,lib_hash=lib_sha,
                 operation=args.action,repeat=rep,ablation=ablation,budget_s=budget)
        if args.action=="witness":
            if not w:
                row["evidence"]={"status":"constructed_invalid_goal","reason":"occupied_goal_reference"}
            else:
                row["evidence"]=invoke(exe,dict(operation="certify",request=r,plan=w,budget_s=30),35)
                row["witness_hash"]=digest(w)
        else:
            payload=dict(operation="solve",request=r,budget_s=budget)
            if ablation:
                payload["options"]=dict(sequence="ASOSA",enable_edits=ablation=="full",
                                        movable_switches=ablation!="fixed")
            row["evidence"]=invoke(exe,payload,budget+7)
        return row
    reps=range(-5,30) if args.action=="timing" else [0]
    conditions=["fixed","movable","full"] if args.action=="ablation" else [None]
    keys=[(t,i,n,c) for t in tasks for i in ids for c in conditions for n in reps]
    with output.open("x") as handle, concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for row in pool.map(trial,keys):
            handle.write(packed(row).decode()+"\n")
            handle.flush()
            e=row["evidence"]
            if args.action=="witness" and row["label"]!="invalid" and e.get("executable") is not True:
                print("Witness validation failed:", row["case_id"], e, file=sys.stderr,flush=True)
            print(row["case_id"],row["repeat"],e.get("status",e.get("executable")),e.get("reason"),flush=True)


    if args.action=="witness":
        for line in output.read_text().splitlines():
            row=json.loads(line)
            if row["label"]!="invalid" and row["evidence"].get("executable") is not True:
                raise SystemExit(1)


if __name__=="__main__":
    main()
