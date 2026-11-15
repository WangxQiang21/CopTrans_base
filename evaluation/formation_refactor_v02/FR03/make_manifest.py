"""Write the FR03 evidence manifest (bytes + sha256 for every FR03 file).

Run from the worktree root after every evidence file exists:

    python3 evaluation/formation_refactor_v02/FR03/make_manifest.py
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parent
OUT = BASE / "evidence_manifest.json"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    files = {}
    for path in sorted(BASE.rglob("*")):
        if not path.is_file():
            continue
        if path.name == OUT.name:
            continue
        relative = path.relative_to(ROOT).as_posix()
        files[relative] = {
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
    payload = {
        "schema": "fr03.evidence_manifest.v1",
        "files": files,
        "file_count": len(files),
    }
    OUT.write_text(json.dumps(payload, indent=1) + "\n")
    print("wrote", len(files), "files to", OUT)


if __name__ == "__main__":
    main()