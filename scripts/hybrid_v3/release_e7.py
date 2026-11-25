#!/usr/bin/env python3
"""Stage a qualified C8 release in a private clone and emit a bundle only after clean-clone verification."""
import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import yaml

from cases import ROOT, SPEC
from default_entries import matches_qualified_runtime
from evidence_manifest import runtime_binaries
from route_evidence import HISTORICAL, matrix

REGISTRY = Path("ros2_ws/src/fwis_bringup/config/test_defaults.yaml")
REPORT = Path("reports/hybrid_v3")
TOOLS = Path("scripts/hybrid_v3")
PROFILES = ("standard", "baseline")
PROFILE_SEALS = {"baseline": "c5d6a27c5fe5a0d27c256a98424dcd8520cebfd70753916d5ed6736ea28346cf",
                 "standard": "1c7cc330fe2dc77e9fee4290d1c66dc596fe9188d198c0560164bd4452840ff9"}
SUITES = ("solve_core", "solve_development", "solve_holdout", "ablation_pruning", "timing_core")
INPUTS = ("batch", "witness", "binary", "sealed", "system", "numeric", "native", "ros", "control", "legacy",
          "gazebo", "headless", "historical", "prior_gazebo", "scenes", "clock_receipts")
COMPACT = {"runs.csv","measurements.json","case_manifest.json","case_manifest_baseline.json","behavior.json",
           "historical_routes.json","legacy_comparison.json","acceptance_spec.yaml"}
DEFAULT = "DEFAULT_INTEGRATION"


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return sha256(path.read_bytes()).hexdigest()


def run(cwd, *args, ok=(0,), env=None):
    result = subprocess.run([str(a) for a in args], cwd=cwd, env=env)
    if result.returncode not in ok:
        raise RuntimeError(f"Exit {result.returncode} from {args[0]} {args[1]}")
    return result.returncode


def git(root, *args):
    return subprocess.check_output(["git", "-C", root, *args], text=True).strip()


def inventory(cfg):
    missing = []
    def required(file):
        if not file.is_file():
            missing.append(str(file))
    for p in PROFILES:
        seal = cfg["sealed"] / f"manifest_{p}.json"
        required(seal)
        if seal.is_file() and sha(seal) != PROFILE_SEALS[p]:
            raise ValueError(f"Changed cohort 4 manifest for {p}")
        for name in SUITES:
            required(cfg["batch"] / f"{name}_{p}.jsonl")
        for split in ("core", "development", "pruning", "holdout"):
            required(cfg["witness"] / f"witness_{split}_{p}.jsonl")
    for key, files in {"system": ("source_commit","ros2_ws/install/setup.bash"),
                       "binary": ("binaries.sha256","sources.sha256","dependencies.txt"),
                       "batch": ("ctest/source_commit",), "numeric": ("verification.json","sanitizer_summary.json"),
                       "native": ("runtime.json",), "legacy": ("previous_only_verified.json","core_timing_references.json"),
                       "ros": ("verification.json","delivery.json","faults.json"),
                       "control": ("1.csv","3.csv","1.summary.json","3.summary.json"),
                       "scenes": ("summary.json",)}.items():
        for f in files:
            required(cfg[key]/f)
    for key in ("historical", "clock_receipts"):
        required(cfg[key])
    for key in ("gazebo","headless"):
        count = len(list(cfg[key].glob("*/metadata.json")))
        expected = 96 if key == "gazebo" else 4
        if count != expected:
            missing.append(f"{key} {count}/{expected} archived trials")
    return missing


def gate(path, *, before_defaults, exit_code):
    checks = read(path)
    expected = {g["id"] for g in yaml.safe_load(SPEC.read_text())["checks"]}
    statuses = {g["id"]: g["status"] for g in checks["checks"]}
    if len(checks["checks"]) != len(expected) or set(statuses) != expected:
        raise ValueError("Incomplete or duplicate mandatory gate list")
    blocked = {k:v for k,v in statuses.items() if v != "PASS"}
    if before_defaults:
        if blocked != {DEFAULT:"FAIL"} or exit_code != 1 or checks["task_state"] != "INCOMPLETE":
            raise ValueError(f"Default transition blocked. Gates: {blocked}")
    elif blocked or exit_code or checks["task_state"] != "COMPLETED":
        raise ValueError(f"Release blocked. Gates: {blocked}")


def promoted_text(original):
    old = yaml.safe_load(original)
    defaults = old["defaults"]
    if defaults["tracking_implementation"] != "cpp":
        raise ValueError("Expected an already qualified C++ tracker")
    new, changes = re.subn(r"(?m)^(defaults:\n  planners:\n)    single: \S+\n    multi: \S+\n",
                           r"\g<1>    single: hybrid_joint\n    multi: hybrid_joint\n", original)
    if changes != 1:
        raise ValueError("Unknown planner registry layout")
    new = re.sub(r"(?m)^  baseline_planners:\n(?:    [^\n]*\n)+", "", new)
    defaults["planners"] = {"single":"hybrid_joint","multi":"hybrid_joint"}
    defaults.pop("baseline_planners",None)
    if yaml.safe_load(new) != old:
        raise ValueError("Change escaped the planner defaults")
    return new


def freeze_checks(cfg):
    solver = (cfg["batch"]/"ctest/source_commit").read_text().strip()
    system = (cfg["system"]/"source_commit").read_text().strip()
    numeric = read(cfg["numeric"]/"verification.json")
    native = read(cfg["native"]/"runtime.json")
    old = read(cfg["legacy"]/"previous_only_verified.json")
    if (numeric["source_commit"] != solver or native["source_commit"] != solver or
            sha(cfg["native"]/"runtime.json") != numeric["raw_sha256"]["native/runtime.json"]):
        raise ValueError("The numeric and native receipts must bind this solver")
    for name, digest in numeric["raw_sha256"].items():
        folder, relative = (cfg["native"], name[7:]) if name.startswith("native/") else (cfg["numeric"],name)
        if sha(folder/relative) != digest:
            raise ValueError("Changed numeric qualification artifact: "+name)
    for variant, evidence in numeric["variants"].items():
        if sha(cfg["numeric"]/variant/"manifest.json") != evidence["manifest_sha256"]:
            raise ValueError("Changed numerical build manifest: "+variant)
    frozen = {Path(k).name:v for k,v in old["sealed_manifest_sha256"].items()}
    if (frozen != {f"manifest_{k}.json":v for k,v in PROFILE_SEALS.items()} or
            old["core_timing_reference_sha256"] != sha(cfg["legacy"]/"core_timing_references.json") or
            old["frozen_holdout_cohort"] != 4):
        raise ValueError("Unmatched legacy cohort or timing reference")
    for key, group in old["cohorts"].items():
        if sha(cfg["legacy"]/f"legacy_{key}.jsonl")!=group["raw_sha256"] or (
                sha(cfg["legacy"]/f"legacy_timing_{key}.jsonl")!=group["timing_raw_sha256"]):
            raise ValueError("Changed previously acquired control observations: "+key)
    for key in ("gazebo","headless"):
        rows = [read(path) for path in cfg[key].glob("*/metadata.json")]
        if not matrix(rows, key == "gazebo") or any(row["source_commit"] != system for row in rows):
            raise ValueError(f"Incomplete, mixed-source or duplicate simulations in {key}")
    scenes = read(cfg["scenes"]/"summary.json")["cases"]
    from qualification_scenes import SPECIAL_SCENES
    needed = {(f,t,p) for f in SPECIAL_SCENES for t in ("single","formation") for p in PROFILES}
    found = {(r["family"],r["task"],r["profile"]) for r in scenes if r["passed"]}
    if not needed.issubset(found):
        raise ValueError("The candidate has incomplete physical scene benchmarks")


def link_batch(src, dst):
    dst.mkdir()
    for f in src.iterdir():
        if f.name != "default_runs.json":
            (dst/f.name).symlink_to(f.resolve(), target_is_directory=f.is_dir())


def prepare_inputs(cfg, work):
    stage = work/"inputs"
    stage.mkdir()
    link_batch(cfg["batch"],stage/"batch")
    legacy = stage/"legacy"
    legacy.mkdir()
    for f in cfg["legacy"].glob("legacy_*.jsonl"):
        (legacy/f.name).symlink_to(f.resolve())
    run(ROOT, sys.executable, TOOLS/"legacy_summary.py", "--sealed", cfg["sealed"], "--directory", cfg["legacy"],
        "--new-batch", cfg["batch"], "--output", legacy/"legacy_progress.json")
    paired = read(legacy/"legacy_progress.json")
    for group in paired["cohorts"].values():
        if not (group["attempts"] == group["unique"] == 120 and group["paired_feasible"]["total"] == 110
                and group["cohorts"] == [4] and group["infrastructure_errors"] == group["false_executable_invalid"] == 0
                and all(t["new_complete"] and t["complete"] for t in group["timing"].values())):
            raise ValueError("Unpaired previous/current observations")
    scenes = stage/"scenes"
    scenes.mkdir()
    (scenes/"summary.json").symlink_to((cfg["scenes"]/"summary.json").resolve())
    for name, args in (("native",["--batch",cfg["batch"],"--cohort","4"]),
                       ("scenes",["--scenes",cfg["scenes"]]),("gui",["--gazebo-batch",cfg["gazebo"]]),
                       ("headless",["--gazebo-batch",cfg["headless"]])):
        target = scenes/"behavior.json" if name=="scenes" else stage/(name+".json")
        run(ROOT,sys.executable,TOOLS/"behavior_metrics.py",*args,"--output",target)
    compact = work/"prefinal"
    compact.mkdir()
    shutil.copy2(SPEC, compact/SPEC.name)
    shutil.copy2(legacy/"legacy_progress.json",compact/"legacy_comparison.json")
    scene_source=cfg["system"]/"ros2_ws/src/ranger_mini_gz"
    catalog_env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(scene_source),os.environ.get("PYTHONPATH","")]))
    run(ROOT,sys.executable,TOOLS/"historical_catalog.py",cfg["historical"],"--output",compact/"historical_routes.json",
        env=catalog_env)
    clocks=stage/"independent_clocks.json"
    run(ROOT,sys.executable,TOOLS/"clock_receipts.py","--gazebo",cfg["gazebo"],"--headless",cfg["headless"],
        "--system",cfg["system"],"--verify",cfg["clock_receipts"],"--output",clocks)
    run(ROOT, sys.executable, TOOLS/"publish_behavior.py","--native",stage/"native.json","--scenes",scenes,
        "--gazebo",stage/"gui.json","--headless",stage/"headless.json","--clock-receipts",clocks,
        "--system",cfg["system"],"--output",compact/"behavior.json")
    return compact, stage


def recompute(root, cfg, compact, inputs, before):
    cmd = ["scripts/accept_hybrid_v3.py","--batch",inputs/"batch","--ctest",cfg["batch"]/"ctest",
           "--legacy",inputs/"legacy","--report",compact]
    for key in ("witness","binary","sealed","system","numeric","native","ros","control","gazebo","headless","historical","prior_gazebo"):
        cmd.extend(("--"+key.replace("_","-"),cfg[key]))
    cmd.extend(("--recorder", cfg.get("recorder",inputs/"no-recorder-override")))
    rc = run(root,sys.executable,*cmd,ok=(0,1))
    gate(compact/"acceptance.json",before_defaults=before,exit_code=rc)
    manifest = read(compact/"manifest.json")
    hashes = manifest["compact_hashes"]
    if not COMPACT.issubset(hashes) or not all(sha(compact/k)==v for k,v in hashes.items()):
        raise ValueError("Incomplete or changed compact files")
    if not before:
        rows = read(inputs/"batch/default_runs.json")
        if not matches_qualified_runtime(rows,runtime_binaries(cfg["system"])):
            raise ValueError("Ordinary default entries differ from the qualified installed binaries")


def relay_archives(clone, cfg):
    runs = clone/"reports/runs"
    original_runs = ROOT/"reports/runs"
    ids = set(HISTORICAL.values()) | {"20261125_140950_e40cIk","20261125_141134_AeDvqX","20261125_141201_TcTcHJ"}
    for key in ("gazebo","headless"):
        ids.update(read(f)["run_id"] for f in cfg[key].glob("*/metadata.json"))
    for name in ids:
        if Path(name).name != name:
            raise ValueError("Invalid experiment identifier")
        original = original_runs/name
        if not original.is_dir():
            raise ValueError("Missing experiment archive: "+name)
    # The actual public launcher acquires the same flock as closed_loop.py
    # and enforces the user's ordinary local experiment quota at this path.
    runs.symlink_to(original_runs,target_is_directory=True)


def run_smoke(clone, output):
    env = os.environ.copy()
    if env.get("COPTRANS_INSTALL_BASE"):
        raise ValueError("Run public smoke with the candidate clone's normal install")
    env["COPTRANS_CASADI_ROOT"] = env.get("COPTRANS_CASADI_ROOT",str(ROOT.parent/".coptrans-python/casadi"))
    env["CMAKE_BUILD_PARALLEL_LEVEL"]="2"
    for task in ("single","multi"):
        for arch in ("optimized","baseline"):
            cmd=[clone/"scripts/test.sh","--dry-run","--task",task,"--architecture-profile",arch]
            for override in ([],["--planner-backend","legacy"]):
                resolved = yaml.safe_load(subprocess.check_output([*cmd,*override],cwd=clone,env=env,text=True))
                expected = ("single" if task=="single" else "formation")+"_hybrid_joint_cpp"
                expected = {"single":"single_state_lattice","multi":"multi_4wis"}[task] if override else expected
                if resolved["planner_backend"] != expected or resolved["tracking_implementation"] != "cpp":
                    raise ValueError(f"Default or explicit legacy mismatch at {task}/{arch}")
    # The simulated RViz input publisher imports geometry_msgs and rclpy before launching.
    run(clone,"bash","-c",'source /opt/ros/humble/setup.bash && exec python3 "$1" --output "$2"',
        "e7",clone/TOOLS/"default_entries.py",output,env=env)


def commit(clone, paths, message):
    run(clone,"git","add","--",*paths)
    run(clone,"git","-c","user.name=CopTrans qualification","-c","user.email=qualification@localhost",
        "commit","-m",message)


def checkout_release_files(clone, revision):
    # Archived ROS 1 blobs in a partial operator checkout may be offline.
    # Hydrate the complete runtime, tools and evidence trees used by E7.
    run(clone,"git","sparse-checkout","init","--cone")
    run(clone,"git","sparse-checkout","set","scripts","ros2_ws/src","reports/hybrid_v3")
    run(clone,"git","checkout","--quiet","--detach",revision)


def publish(cfg, work, ref):
    work.mkdir(parents=True,exist_ok=False)
    freeze_checks(cfg)
    base=git(ROOT,"rev-parse",ref+"^{commit}")
    # Qualification tools and candidate code must be exactly the selected revision, ignoring local raw files.
    if git(ROOT,"status","--porcelain","--untracked-files=all","--","scripts","ros2_ws/src") or git(ROOT,"rev-parse","HEAD")!=base:
        raise ValueError("Qualification code or production source differs from the requested packaging commit")
    compact, stage = prepare_inputs(cfg, work)
    recompute(ROOT,cfg,compact,stage,True)
    clone=work/"candidate"
    run(ROOT,"git","clone","--quiet","--no-checkout","--shared",ROOT,clone)
    checkout_release_files(clone,base)
    (clone/REGISTRY).write_text(promoted_text((clone/REGISTRY).read_text()))
    commit(clone,[REGISTRY],"Use certified hybrid planners for default simulations")
    relay_archives(clone,cfg)
    run_smoke(clone,stage/"batch/default_runs.json")
    # The versioned report is regenerated in an isolated clone. Old report data cannot fill missing artifacts.
    out=clone/REPORT
    shutil.rmtree(out)
    shutil.copytree(compact,out)
    recompute(clone,cfg,out,stage,False)
    commit(clone,[REPORT],"Publish verified hybrid planning and control evidence")
    clean=work/"clean"
    run(clone,"git","config","uploadpack.allowFilter","true")
    run(clone,"git","config","uploadpack.allowAnySHA1InWant","true")
    run(ROOT,"git","clone","--quiet","--no-local","--no-checkout","--filter=blob:none",clone,clean)
    checkout_release_files(clean,git(clone,"rev-parse","HEAD"))
    run(clean,"git","remote","remove","origin")
    # This clone contains neither local log directories nor the simulation install.
    if any((clean/name).exists() for name in ("reports/runs","ros2_ws/install",".git/objects/info/alternates")):
        raise ValueError("Unexpected raw artifacts in clean verifier")
    expected=sha(clean/REPORT/"acceptance.json")
    rc=run(clean,sys.executable,"scripts/accept_hybrid_v3.py","--compact")
    gate(clean/REPORT/"acceptance.json",before_defaults=False,exit_code=rc)
    if sha(clean/REPORT/"acceptance.json")!=expected or git(clean,"status","--porcelain"):
        raise ValueError("Clean checkout could not reproduce the published acceptance")
    bundle=work/"release.bundle"
    run(clone,"git","bundle","create",bundle,"HEAD","^"+base)
    run(ROOT,"git","bundle","verify",bundle)
    print(f"Verified candidate {git(clone,'rev-parse','HEAD')} on {base}; bundle {bundle}",flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,required=True,help="JSON object with explicit evidence paths; relative paths use the repository root")
    parser.add_argument("--stage",type=Path,help="fresh staging directory, required to run public smoke after the evidence passes")
    parser.add_argument("--packaging-ref",help="committed release code to package, required with --stage")
    args=parser.parse_args()
    cfg={k:(ROOT/Path(v)).absolute() for k,v in read(args.config).items()}
    missing_keys=set(INPUTS)-cfg.keys()
    if missing_keys:parser.error("Missing paths: "+", ".join(sorted(missing_keys)))
    missing=inventory(cfg)
    if missing:
        print("Release evidence incomplete:",*missing,sep="\n",file=sys.stderr)
        return 1
    if args.stage:
        if not args.packaging_ref:parser.error("Supply --packaging-ref with --stage")
        publish(cfg,args.stage.resolve(),args.packaging_ref)
    else:
        print("Required files are present. Only --stage runs the gate sequence and public defaults.")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
