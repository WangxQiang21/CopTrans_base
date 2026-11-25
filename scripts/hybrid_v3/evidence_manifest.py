"""Identify the immutable machine runs behind the compact qualification results."""
import re
import hashlib
from pathlib import Path

import subprocess

from qualify import ROOT, POLICY, digest, sha, load
from cases import GEOMETRY, REGISTRY, REPORT, SPEC

RANKING_SOURCE="ros2_ws/src/fwis_hybrid_joint/config/policy.json"
RECORDER_SOURCE="ros2_ws/src/fwis_bringup/scripts/run_experiment"
CONFIG_FILES=(REGISTRY,GEOMETRY,SPEC,ROOT/RANKING_SOURCE,
              ROOT/"ros2_ws/src/fwis_backends/multi_4wis/config/transport_formation.yaml",
              ROOT/"ros2_ws/src/ranger_mini_gz/config/cargo_geometry.yaml")
COMPACT_REQUIRED={"measurements.json","runs.csv","case_manifest.json","case_manifest_baseline.json",
                  "historical_routes.json","legacy_comparison.json","behavior.json","acceptance_spec.yaml"}


def qualification_inputs(case_manifests):
    # Canonical keys identify the inputs in any checkout. Case bytes come from
    # the manifests shipped with this report, including staged release reports.
    inputs={str(p.relative_to(ROOT)):sha(p) for p in CONFIG_FILES}
    for path in case_manifests.values():
        inputs[str((REPORT/path.name).relative_to(ROOT))]=sha(path)
    return inputs


def compact_inputs_match(out, manifest, recorded, case_manifests):
    expected=qualification_inputs(case_manifests)
    configs={str(p.relative_to(ROOT)):expected[str(p.relative_to(ROOT))] for p in CONFIG_FILES}
    cases={p.name:sha(p) for p in case_manifests.values()}
    files=manifest.get("compact_hashes",{})
    return (recorded==expected and manifest.get("config_hashes")==configs and
            manifest.get("case_sha256")==cases and COMPACT_REQUIRED<=files.keys() and
            all(not Path(name).is_absolute() and ".." not in Path(name).parts and
                (out/name).is_file() and sha(out/name)==value for name,value in files.items()))


def read_hash_file(path):
    if not path.is_file():
        return {}
    result={}
    for line in path.read_text().splitlines():
        digest,name=line.split(None,1)
        if not re.fullmatch("[0-9a-f]{64}",digest) or name in result:
            raise ValueError(f"Invalid SHA-256 list at {path}")
        result[name]=digest
    return result


def matches_source(name,expected,commit):
    local=ROOT/name
    if local.is_file() and sha(local)==expected:
        return True
    if not commit:
        return False
    obj=subprocess.run(["git","show",commit+":"+name],cwd=ROOT,capture_output=True)
    return obj.returncode==0 and hashlib.sha256(obj.stdout).hexdigest()==expected


def current_sources_match(manifest):
    sources=manifest.get("source_sha256",{}) | manifest.get("system_source_sha256",{})
    if verified_recorder(manifest):
        sources[RECORDER_SOURCE]=manifest["recorder_revision"]["build_and_runtime"]["new_runner"]["sha256"]
    return bool(sources) and all((ROOT/n).is_file() and sha(ROOT/n)==v for n,v in sources.items())


def build_manifest(batch, witnesses, binary_dir, ctest_dir):
    binary=read_hash_file(binary_dir/"binaries.sha256")
    # Colcon adjusts the installed RUNPATH. Record the library actually resolved
    # by the frozen executable as well as the build artifact.
    dependencies=binary_dir/"dependencies.txt"
    if dependencies.is_file():
        loaded=re.search(r"libfwis_hybrid_joint\.so => (.+?) \(",dependencies.read_text())
        if loaded and Path(loaded[1]).is_file():
            binary[loaded[1]]=sha(Path(loaded[1]))
    sources=read_hash_file(binary_dir/"sources.sha256")
    inputs=[*batch.glob("*.jsonl"),*witnesses.glob("*.jsonl"),*ctest_dir.glob("*.xml")]
    binaries_verified=bool(binary) and all(Path(name).is_file() and sha(Path(name))==value for name,value in binary.items())
    commit=(ctest_dir/"source_commit").read_text().strip() if (ctest_dir/"source_commit").is_file() else None
    source_verified=bool(sources) and all(matches_source(name,value,commit) for name,value in sources.items())
    return dict(binary_sha256=binary,source_sha256=sources,verified_at_collection=binaries_verified and source_verified,
                raw_sha256={str(p.resolve()):sha(p) for p in sorted(inputs)},
                binary_commit=commit)


def system_hashes(commit):
    if not commit:
        return {}
    scopes=("ros2_ws/src/fwis_hybrid_joint_ros","ros2_ws/src/fwis_v3_msgs/msg","ros2_ws/src/fwis_control/src",
            "ros2_ws/src/fwis_control/include","ros2_ws/src/fwis_bringup/launch/test.launch.py",
            "ros2_ws/src/fwis_bringup/launch/plan.launch.py","ros2_ws/src/fwis_bringup/fwis_bringup",
            "ros2_ws/src/fwis_bringup/scripts","ros2_ws/src/ranger_mini_gz/ranger_mini_gz",
            "ros2_ws/src/ranger_mini_gz/src","ros2_ws/src/ranger_mini_gz/worlds",
            "ros2_ws/src/ranger_mini_gz/config","ros2_ws/src/fwis_control/config")
    paths=subprocess.check_output(["git","ls-tree","-r","--name-only",commit,"--",*scopes],cwd=ROOT,text=True).splitlines()
    from hashlib import sha256
    return {name:sha256(subprocess.check_output(["git","show",commit+":"+name],cwd=ROOT)).hexdigest() for name in paths
            if "/test/" not in name and not name.endswith((".md",".rst"))}


def runtime_binaries(snapshot):
    install=snapshot/"ros2_ws/install"
    paths=[install/name/"lib"/name/node for name,node in (
        ("fwis_hybrid_joint_ros","hybrid_joint_planner"),
        ("fwis_hybrid_joint_ros","hybrid_joint_executor"),
        ("fwis_control","ranger_tracking_controller_cpp"))]
    paths+=[install/name/"lib"/lib for name,lib in (("fwis_hybrid_joint","libfwis_hybrid_joint.so"),
                                                    ("fwis_hybrid_joint_ros","libhybrid_codec.so"))]
    # Keep the recorded installation path even when its build cache is relocated.
    # Reading it still hashes the full binary at the link destination.
    return {str(p.absolute()):sha(p) for p in paths if p.is_file()}


def solver_system_compatibility(manifest):
    """The system can change its clock path while deploying the identical certified solver."""
    versions=(manifest.get("binary_commit"),manifest.get("system_commit"))
    if not all(versions):
        return {}
    package="ros2_ws/src/fwis_hybrid_joint"
    trees=[subprocess.check_output(["git","rev-parse",v+":"+package],cwd=ROOT,text=True).strip() for v in versions]
    libs=[]
    for group in ("binary_sha256","runtime_binaries"):
        libs.append(sorted({value for name,value in manifest.get(group,{}).items() if Path(name).name=="libfwis_hybrid_joint.so"}))
    return dict(solver_commit=versions[0],system_commit=versions[1],package=package,
                solver_tree=trees[0],system_tree=trees[1],solver_libraries=libs[0],system_libraries=libs[1])


def same_solver(manifest):
    match=manifest.get("solver_system_compatibility") or {}
    return bool(match.get("solver_commit")==manifest.get("binary_commit") and match.get("system_commit")==manifest.get("system_commit")
                and match.get("solver_tree") and match["solver_tree"]==match.get("system_tree")
                and len(match.get("solver_libraries",[]))==1 and match["solver_libraries"]==match.get("system_libraries"))


def observed_runtime_sources(system, group, control):
    """Read the installed process digests signed into the raw timing and DDS captures."""
    observed={}
    bench=system/"ros2_ws/build/fwis_control/hybrid_deadlines"
    if bench.is_file():
        actual=sha(bench)
        observed["load"]={"build_sha256":actual,"runs":{}}
        for n in (1,3):
            saved=read_hash_file(control/f"{n}.binary.sha256")
            csv=control/f"{n}.csv"
            summary=control/f"{n}.summary.json"
            if saved and csv.is_file() and summary.is_file() and summary.stat().st_size:
                observed["load"]["runs"][str(n)]={"captured_sha256":list(saved.values()),"raw_csv_sha256":sha(csv),
                                                   "summary_sha256":sha(summary)}
    faults=load(group/"faults.json") or {}
    if faults.get("raw_directory"):
        raw=Path(faults["raw_directory"])
        before,after=(raw/n for n in ("binaries.before.sha256","binaries.after.sha256"))
        files=read_hash_file(before)
        verification=load(group/"verification.json") or {}
        parity_dir=faults.get("parity_directory") or verification.get("parity_directory")
        checked={}
        for name,expected in faults.get("raw_file_hashes",{}).items():
            f=raw/name
            if not f.is_file() and parity_dir:
                f=Path(parity_dir)/name
            if f.is_file() and sha(f)==expected:
                checked[name]=expected
        observed["dds"]={"actual_files":files,"capture_sha256":sha(before),"after_sha256":sha(after),
                         "recorded_sha256":faults["binary_manifest_sha256"],"complete_raw":bool(checked) and checked==faults["raw_file_hashes"],
                         "source_commit":(raw/"source_commit").read_text().strip(),"checked_sha256":checked}
    return observed


def runtime_sources_match(manifest,control,kind):
    evidence=(manifest.get("observed_runtime_sources") or {}).get(kind,{})
    if kind=="dds":
        return bool(evidence.get("actual_files") and evidence["actual_files"]==manifest.get("runtime_binaries") and
                    evidence["source_commit"]==manifest.get("system_commit") and evidence["complete_raw"] and
                    evidence["capture_sha256"]==evidence["after_sha256"]==evidence["recorded_sha256"])
    benchmark=evidence.get("build_sha256")
    rows=evidence.get("runs",{})
    return bool(benchmark and set(rows)=={"1","3"} and set(control)=={"1/1","3/1","3/2","3/3"} and all(
        row["captured_sha256"]==[benchmark] and row["raw_csv_sha256"]==control[f"{n}/1"]["csv_sha256"]
        for n,row in rows.items()))


def recorder_revision(path):
    """Bind the later archive-only change to its paired observed fault tests."""
    receipt=load(path)
    if not receipt or receipt["status"]!="validated":
        return None
    def check_files(value):
        if isinstance(value,dict):
            if "file" in value and "sha256" in value:
                p=Path(value["file"])
                if not p.is_file() or sha(p)!=value["sha256"]:
                    raise ValueError("Changed recorder evidence: "+str(p))
            for v in value.values(): check_files(v)
        elif isinstance(value,list):
            for v in value: check_files(v)
    check_files(receipt)
    return receipt


def verified_recorder(manifest):
    receipt=manifest.get("recorder_revision") or {}
    versions=receipt.get("build_and_runtime",{})
    old=manifest.get("system_source_sha256",{}).get(RECORDER_SOURCE)
    pairs=receipt.get("paired_runs",{})
    if not (receipt.get("status")=="validated" and versions.get("native_source_commit")==manifest.get("system_commit")
            and versions.get("native_binaries")==manifest.get("runtime_binaries") and old
            and versions.get("old_runner",{}).get("sha256")==old
            and versions.get("new_runner",{}).get("sha256")==sha(ROOT/RECORDER_SOURCE)
            and versions.get("unit_suites",{}).get("stop_observation")
            and set(pairs)=={"original","fixed"}
            and all({r["task"] for r in pairs[k]}=={"single","formation"} for k in pairs)):
        return False
    for r in pairs["fixed"]:
        if r.get("reported_status")!="stopped" or not r.get("termination_observation",{}).get("final",{}).get("window_complete"):
            return False
        for car in r["cars"].values():
            if not (car["result_cmd_vel"]==car["independent_cmd_vel_last"]["velocity"]==[0.,0.,0.] and
                    car["result_last_odom_twist"]==car["independent_odom_last"]["velocity"]==[0.,0.,0.]):
                return False
    return True


def audit_rows(records,witnesses,manifest_files,source):
    manifests={k:{r["id"]:r for r in (load(v) or {}).get("cases",[])} for k,v in manifest_files.items()}
    reasons=[]
    if not source or not source.get("verified_at_collection") or not source.get("raw_sha256") or not source.get("binary_commit"):
        reasons.append("missing_source_or_raw_identity")
    sources=(source or {}).get("source_sha256",{}) | (source or {}).get("system_source_sha256",{})
    if not source or not source.get("system_commit") or not source.get("system_source_sha256"):
        reasons.append("missing_system_identity")
    if source and verified_recorder(source):
        sources[RECORDER_SOURCE]=source["recorder_revision"]["build_and_runtime"]["new_runner"]["sha256"]
    for name,value in sources.items():
        commit=source["system_commit"] if name in source.get("system_source_sha256",{}) else source["binary_commit"]
        if not matches_source(name,value,commit):
            reasons.append("source/"+name)
    allowed=set((source or {}).get("binary_sha256",{}).values())
    seen=set()
    for row in records:
        identity=(row["profile"],row["case_id"],row["operation"],row["ablation"],row["repeat"])
        declared=manifests.get(row["profile"],{}).get(row["case_id"],{})
        mismatch=(row["request_hash"]!=declared.get("request") or row["map_hash"]!=declared.get("map")
                  or row["limits_hash"]!=declared.get("limits") or row["label"]!=declared.get("label")
                  or row["family"]!=declared.get("family") or row["seed"]!=declared.get("seed")
                  or row["exe_hash"] not in allowed or row["lib_hash"] not in allowed or not row["config_hash"])
        expected_config=digest(dict(policy=POLICY,ranking_sha256=sources.get(RANKING_SOURCE),
                                    limits_hash=row["limits_hash"],task=row["task"],ablation=row["ablation"]))
        mismatch |= (row.get("ranking_policy_sha256")!=sources.get(RANKING_SOURCE) or row["config_hash"]!=expected_config
                     or row.get("source_sha")!=(source or {}).get("binary_commit"))
        if identity in seen or mismatch:
            reasons.append(str(identity))
        seen.add(identity)
    for profile,manifest in manifests.items():
        for cid,declared in manifest.items():
            receipt=witnesses.get(cid+"/"+profile)
            if not receipt or receipt[0]!=declared["request"] or receipt[2]!=declared["witness"]:
                reasons.append("witness/"+profile+"/"+cid)
    return dict(verified=bool(records) and not reasons,attempts=len(seen),issues=len(reasons),first_issues=reasons[:10])
