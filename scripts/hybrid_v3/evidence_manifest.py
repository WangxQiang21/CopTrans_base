"""Identify the immutable machine runs behind the compact qualification results."""
import re
from pathlib import Path

import subprocess

from qualify import ROOT, POLICY, digest, sha, load

RANKING_SOURCE="ros2_ws/src/fwis_hybrid_joint/config/policy.json"


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
    source_verified=bool(sources) and all((ROOT/name).is_file() and sha(ROOT/name)==value for name,value in sources.items())
    return dict(binary_sha256=binary,source_sha256=sources,verified_at_collection=binaries_verified and source_verified,
                raw_sha256={str(p.resolve()):sha(p) for p in sorted(inputs)},
                binary_commit=(ctest_dir/"source_commit").read_text().strip() if (ctest_dir/"source_commit").is_file() else None)


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
    return {str(p.resolve()):sha(p) for p in paths if p.is_file()}


def audit_rows(records,witnesses,manifest_files,source):
    manifests={k:{r["id"]:r for r in (load(v) or {}).get("cases",[])} for k,v in manifest_files.items()}
    reasons=[]
    if not source or not source.get("verified_at_collection") or not source.get("raw_sha256") or not source.get("binary_commit"):
        reasons.append("missing_source_or_raw_identity")
    sources=(source or {}).get("source_sha256",{}) | (source or {}).get("system_source_sha256",{})
    if not source or not source.get("system_commit") or not source.get("system_source_sha256"):
        reasons.append("missing_system_identity")
    for name,value in sources.items():
        if not (ROOT/name).is_file() or sha(ROOT/name)!=value:
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
