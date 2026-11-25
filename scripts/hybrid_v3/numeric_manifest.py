"""Bind unmodified native inputs, binaries, build flags, and result files."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]


def sha(p):
    with p.open("rb") as handle:
        h=hashlib.sha256()
        for block in iter(lambda:handle.read(1<<20),b""):
            h.update(block)
        return h.hexdigest()


def input_hashes(source_root=ROOT):
    sources={}
    for pkg in ("fwis_planning_core","fwis_hybrid_joint"):
        base=source_root/"ros2_ws/src"/pkg
        files=[base/"package.xml",base/"CMakeLists.txt",*base.glob("config/*.json"),*base.glob("src/*"),*base.glob("include/*/*"),
               *base.glob("test/*.cpp"),*base.glob("benchmark/*.cpp")]
        sources.update({str(p.relative_to(source_root)):sha(p) for p in files if p.is_file()})
    return sources


def main():
    args=argparse.ArgumentParser(description=__doc__)
    args.add_argument("directory",type=Path)
    args.add_argument("--freeze",action="store_true")
    a=args.parse_args()
    out=a.directory.resolve()
    inputs=out/"build_inputs.json"
    if a.freeze:
        sources=input_hashes()
        identifier=hashlib.sha256(json.dumps(sources,sort_keys=True).encode()).hexdigest()
        snapshot=out/"sources"/identifier
        for pkg in ("fwis_planning_core","fwis_hybrid_joint"):
            relative=Path("ros2_ws/src")/pkg
            shutil.copytree(ROOT/relative,snapshot/relative,dirs_exist_ok=True)
        if input_hashes(snapshot)!=sources:
            raise SystemExit("Source changed while freezing it; rerun to take a consistent snapshot")
        inputs.write_text(json.dumps(dict(source_sha=subprocess.check_output(["git","rev-parse","HEAD"],
             cwd=ROOT,text=True).strip(),snapshot=str(snapshot),identifier=identifier,files=sources),sort_keys=True)+"\n")
        print(identifier)
        return
    frozen=json.loads(inputs.read_text());sources=frozen["files"]
    snapshot=Path(frozen["snapshot"]);build=out/"build"/frozen["identifier"]
    if input_hashes(snapshot)!=sources:raise SystemExit("Frozen source tree changed during build")
    results={}
    for name in ("core.xml","joint.xml","model.json","collision.json","derivatives.json","model.csv","collision.csv"):
        f=out/name
        results[name]={"sha256":sha(f),"bytes":f.stat().st_size}
        if f.suffix==".json": results[name]["measurements"]=json.loads(f.read_text())
        elif f.suffix==".xml":
            doc=ET.parse(f)
            results[name]["tests"]=[dict(name=t.attrib["name"],
                passed=t.get("status")=="run" and not any(t.find(tag) is not None for tag in ("failure","error","skipped")))
                for t in doc.findall(".//testcase")]
    binaries={}
    for package in ("core","joint"):
        for f in (build/package).iterdir():
            if f.is_file():
                with f.open("rb") as handle:
                    if handle.read(4)==b"\x7fELF":
                        binaries[str(f.relative_to(out))]=sha(f)
    manifests=dict(schema="coptrans.v3.native_numeric.v1",source_sha=frozen["source_sha"],
        source_file_sha256=sources,frozen_tree=str(snapshot),worktree_matches=frozen["files"]==input_hashes(),
        python_bindings_built=False,build_directory=str(out),host=platform.platform(),compiler=subprocess.check_output(
        ["c++","--version"],text=True).splitlines()[0],results=results,binary_sha256=binaries,
        caches={name:sha(build/name/"CMakeCache.txt") for name in ("core","joint")})
    manifest_file=out/"manifest.json"
    manifest_file.write_text(json.dumps(manifests,indent=2)+"\n")
    flags=(build/"joint"/"CMakeCache.txt").read_text()
    if "-fsanitize=address,undefined" in flags:
        suites=[t for k in ("core.xml","joint.xml") for t in results[k]["tests"]]
        required={"hybrid_solver_test","hybrid_solver_smoke"}
        tsan_file=out.parent/"tsan_host.txt"
        tsan=tsan_file.read_text().strip() if tsan_file.exists() else None
        summary=dict(sanitizers=["address","undefined"],native_solve_passed=required.issubset(
                     {t["name"] for t in suites if t["passed"]}),all_groups_passed=bool(suites) and all(t["passed"] for t in suites),
                     executed_groups=suites,leak_detection=True,tsan=tsan,
                     manifest_path=str(manifest_file),manifest_sha256=sha(manifest_file),
                     tsan_probe_sha256=(sha(tsan_file) if tsan_file.exists() else None))
        (out.parent/"sanitizer_summary.json").write_text(json.dumps(summary,indent=2)+"\n")


if __name__=="__main__":main()
