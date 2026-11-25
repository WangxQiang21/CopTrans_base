"""Write compact measured scalars and environment identifiers. Full runs stay in local storage."""
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
from cases import ROOT,SPEC,REGISTRY,GEOMETRY,REPORT,LOCAL,POLICY
from qualify import sha


def command(*args):
    return subprocess.check_output(args,cwd=ROOT,text=True).strip()


def export(out,data):
    fields=("case_id","dataset_split","task","family","seed","profile","operation","repeat","ablation","status",
            "reason","actual_backend","exe_hash","lib_hash","source_sha","source_tree","config_hash","map_hash",
            "limits_hash","request_hash","valid","first_certified_s","first_policy_compliant_s","wall_s","duration_s",
            "deadline_overrun_s","result_hash","mode_sequence","aux_blocks","oblique_distance_m","total_spin_rad",
            "a_length_m","longest_A_m","internal_stops","curvature_jump","terminal_position_m","constrained_terminal_yaw_rad")
    with (out/"runs.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fields,extrasaction="ignore");w.writeheader()
        for row in data["cases"]:w.writerow(dict(row,**row["metrics"]))
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
                  raw_local=str(LOCAL),reference_run_ids=["20261125_140950_e40cIk","20261125_141134_AeDvqX","20261125_141201_TcTcHJ"],
                  compact_hashes={p.name:sha(p) for p in [out/"runs.csv",out/"measurements.json",out/"case_manifest.json"] if p.exists()})
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+"\n")
