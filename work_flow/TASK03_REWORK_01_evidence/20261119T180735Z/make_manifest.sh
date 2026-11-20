#!/usr/bin/env bash
# SHA256 manifest of every evidence file under this directory (except the
# manifest outputs themselves and Python bytecode caches).
set -u
E=/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/TASK03_REWORK_01_evidence/20261119T180735Z
cd "$E" || exit 1
find . -type f \
  ! -name 'manifest.sha256' ! -name 'manifest_summary.json' \
  ! -path '*/__pycache__/*' ! -name '*.pyc' \
  -print0 | sort -z | xargs -0 sha256sum > manifest.sha256
python3 - "$E" <<'PY'
import hashlib
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
files = [
    path for path in sorted(root.rglob("*"))
    if path.is_file()
    and path.name not in ("manifest.sha256", "manifest_summary.json")
    and "__pycache__" not in path.parts
    and path.suffix != ".pyc"
]
summary = {
    "schema": "task03_rework01.evidence_manifest.v1",
    "root": str(root),
    "file_count": len(files),
    "total_bytes": sum(path.stat().st_size for path in files),
    "manifest_sha256": hashlib.sha256(
        (root / "manifest.sha256").read_bytes()).hexdigest(),
    "top_level": sorted(
        str(path.relative_to(root)) for path in root.iterdir()
        if path.name not in ("manifest.sha256", "manifest_summary.json")),
}
(root / "manifest_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2, sort_keys=True))
PY
echo DONE