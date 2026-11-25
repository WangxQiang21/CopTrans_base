"""Write compact measured scalars and environment identifiers. Full runs stay in local storage."""
import csv
import gzip
from hashlib import sha256
import json
import os
from pathlib import Path
import platform
import subprocess
from cases import ROOT,SPEC,REGISTRY,GEOMETRY,REPORT,LOCAL,POLICY,make,packed
from qualify import sha


def command(*args):
    return subprocess.check_output(args,cwd=ROOT,text=True).strip()


def representative_plans(out,batch,necessary):
    # Only generated test inputs belong in Git. All operator experiment models
    # and telemetry remain in the local archive, regardless of run outcome.
    selected={("single","C03"),("formation","C06"),("formation","C10")}
    folder=out/"representative_plans"
    for path in batch.glob("solve_core_standard.jsonl"):
        with path.open() as stream:
            for line in stream:
                r=json.loads(line)
                key=(r["task"],r["family"])
                if key not in selected or not r["evidence"].get("audit",{}).get("executable"):
                    continue
                req,_,_=make("core",r["index"],r["task"],"standard")
                document=dict(case=r["case_id"],profile="standard",request=req,request_sha256=r["request_hash"],
                              executable_sha256=r["exe_hash"],solver_sha256=r["lib_hash"],result=r["evidence"])
                folder.mkdir(exist_ok=True)
                (folder/("_".join(key)+".json.gz")).write_bytes(gzip.compress(packed(document),mtime=0))
                selected.remove(key)
    # This construction includes a mathematical lower bound excluding any pure A path.
    if necessary:
        from necessary_aux import required_tasks
        requests={task:req for task,req,_,_ in required_tasks()}
        for item in necessary.get("cases",[]):
            req=requests[item["task"]]
            if item.get("passed") and item["request_sha"]==sha256(packed(req)).hexdigest():
                folder.mkdir(exist_ok=True)
                document={"task":item["task"],"request":req,"request_sha256":item["request_sha"],"bound":item["bound"],
                          "result":item["solve"],"independent_reference_audit":item["witness_audit"]}
                (folder/(item["task"]+"_necessary_aux.json.gz")).write_bytes(gzip.compress(packed(document),mtime=0))


def export(out,data,batch):
    fields=("run_id","schema","build_type","legacy_fallback_used","mode_sequence_before","certificate","stage_times",
            "publish_s","publication_to_receive_s","request_to_receive_s","delivery_and_admission_wall_s","max_mount_error_m","max_terminal_position_m","start_skew_s","case_id","dataset_split","task","family","seed","profile","operation","repeat","ablation","status",
            "reason","actual_backend","exe_hash","lib_hash","source_sha","source_tree","config_hash","map_hash",
            "limits_hash","request_hash","valid","first_certified_s","first_policy_compliant_s","wall_s","duration_s",
            "deadline_overrun_s","result_hash","mode_sequence","aux_blocks","oblique_distance_m","total_spin_rad",
            "a_length_m","longest_A_m","internal_stops","curvature_jump","terminal_position_m","constrained_terminal_yaw_rad")
    with (out/"runs.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fields,extrasaction="ignore");w.writeheader()
        for row in data["cases"]:
            item={**row,**row["metrics"],"schema":"coptrans.hybrid_joint.benchmark/v3","build_type":"Release",
                  "run_id":"/".join(str(row[k]) for k in ("profile","case_id","operation","ablation","repeat")),
                  "legacy_fallback_used":False}
            for name in ("certificate","stage_times"):
                item[name]=json.dumps(item[name],separators=(",",":")) if item.get(name) else None
            w.writerow(item)
        for row in data["gazebo"]:
            metrics=row.get("metrics") or {}
            adm=(row.get("admissions") or [{}])[0]
            endpoints=[v.get("terminal_position_error_m") for v in (metrics.get("vehicles") or {}).values()]
            w.writerow(dict(run_id=row["run_id"],task=row["task"],family=row["family"],operation="gazebo",
                       dataset_split="closed_loop",profile=row["profile"],repeat=row["trial"],status=row["status"],
                       reason=row["reason"],actual_backend=row["backend"],source_sha=row.get("source_sha"),build_type="Release",
                       schema="hybrid_joint_transport/v1",map_hash=row.get("map_sha256"),config_hash=row.get("config_sha256"),
                       legacy_fallback_used=False,publish_s=row.get("publish_s"),publication_to_receive_s=row.get("publication_to_receive_s"),request_to_receive_s=row.get("request_to_receive_s"),duration_s=row.get("plan_duration_s"),
                       delivery_and_admission_wall_s=adm.get("delivery_and_admission_wall_s"),max_mount_error_m=metrics.get("max_mount_error_m"),
                       max_terminal_position_m=max((v for v in endpoints if v is not None),default=None),start_skew_s=metrics.get("start_skew_s"),
                       result_hash=row["result_sha256"]))
    representative_plans(out,batch,data.get("necessary_aux"))
    cpu=next(ln.split(":",1)[1].strip() for ln in Path("/proc/cpuinfo").read_text().splitlines() if ln.startswith("model name"))
    path=Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    manifest=dict(base_commit="ef51bebb112a2efd79b2d7d8b69f9998f0fdf3d8",
                  initial_worktree_dirty=False,source_head=command("git","rev-parse","HEAD"),branch=command("git","branch","--show-current"),
                  source_tree=command("python3",str(ROOT/"scripts/source_fingerprint.py"),str(ROOT)),
                  cpu=cpu,governor=path.read_text().strip() if path.is_file() else None,affinity=sorted(os.sched_getaffinity(0)),
                  os=platform.platform(),compiler=command("c++","--version").splitlines()[0],build_type="Release",
                  dependencies=dict(casadi="3.7.2",ROS="Humble"),policy=POLICY,
                  threads=dict(OPENBLAS_NUM_THREADS=1,OMP_NUM_THREADS=1),scope="static_map_simulation_and_native_soft_realtime_load",
                  config_hashes={str(p.relative_to(ROOT)):sha(p) for p in (REGISTRY,GEOMETRY,SPEC)},
                  case_sha256={p:sha(REPORT/p) for p in ("case_manifest.json","case_manifest_baseline.json")},
                  numeric_and_control_sources=data.get("source_manifest"),
                  raw_local=str(LOCAL),reference_run_ids=["20261125_140950_e40cIk","20261125_141134_AeDvqX","20261125_141201_TcTcHJ"],
                  compact_hashes={p.name:sha(p) for p in [out/"runs.csv",out/"measurements.json",out/"case_manifest.json",out/"case_manifest_baseline.json"] if p.exists()})
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+"\n")
