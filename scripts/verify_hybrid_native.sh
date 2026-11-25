#!/usr/bin/env bash
set -eo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
evidence=${1:?provide a local evidence directory}
mkdir -p "$evidence"
evidence=$(realpath "$evidence")
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
build=$(mktemp -d "${TMPDIR:-/tmp}/coptrans-native.XXXXXXXX")
trap 'rm -rf -- "$build"' EXIT
cd "$root/ros2_ws"
colcon --log-base "$build/log" build --packages-up-to fwis_hybrid_joint --install-base "$build/install" \
  --build-base "$build/build" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DFWIS_PLANNING_CORE_PYTHON=OFF > "$evidence/build.log" 2>&1
source "$build/install/setup.bash"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
for pkg in fwis_planning_core fwis_hybrid_joint; do
  ctest --test-dir "$build/build/$pkg" --output-on-failure --output-junit "$evidence/$pkg.xml"
done
bin="$build/install/fwis_hybrid_joint/lib/fwis_hybrid_joint/hybrid_joint_case"
cp "$build/build/fwis_planning_core/CMakeCache.txt" "$evidence/core-cache.txt"
ldd "$bin" > "$evidence/libraries.txt"
cd "$root"
# Only the benchmark harness constructs inputs; strace starts at the native executable.
python3 - "$evidence" <<'PY'
import json,sys
from pathlib import Path
sys.path.insert(0,"scripts/hybrid_v3")
from cases import make
out=Path(sys.argv[1])/"native.in"
with out.open("w") as f:
    for task in ("single","formation"):
        req,*_=make("core",0,task,"standard")
        print(json.dumps(dict(operation="solve",request=req,budget_s=8 if task=="single" else 12)),file=f)
PY
strace -f -e trace=execve,execveat,openat -o "$evidence/trace.txt" \
  "$bin" < "$evidence/native.in" > "$evidence/native.jsonl"
python3 - "$evidence" <<'PY'
from pathlib import Path
import json,re,sys
p=Path(sys.argv[1])
rows=list(map(json.loads,(p/"native.jsonl").read_text().splitlines()))
trace=(p/"trace.txt").read_text().splitlines()
execs=[x for x in trace if re.search(r"\bexecve(at)?\(",x)]
bad=[x for x in trace if re.search(r'/(libpython[^/"]*\.so|python[.0-9]*"|[^/"]+\.(py|pyc)")',x)]
result=dict(traced_execs=execs,forbidden_loads=bad,backend=[x.get("actual_backend") for x in rows],
            certified=[x.get("audit",{}).get("executable",False) for x in rows],
            pybind_disabled="FWIS_PLANNING_CORE_PYTHON:BOOL=OFF" in (p/"core-cache.txt").read_text())
(p/"runtime.json").write_text(json.dumps(result,indent=2))
if bad or len(execs)!=1 or len(rows)!=2 or not all(result["certified"]) or not result["pybind_disabled"]:
    raise SystemExit(1)
PY
