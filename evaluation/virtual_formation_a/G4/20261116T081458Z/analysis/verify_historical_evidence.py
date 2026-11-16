"""Verify that the previous G4 run directory is byte-for-byte unchanged.

The old G4 source manifest recorded every run file except its own three
self-referential artifacts.  This script reloads that manifest, re-hashes every
recorded file in the old run, and reports any missing, resized or modified
file.  It never writes into the historical directory.
"""
import hashlib
import json
from pathlib import Path

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
OLD_RUN = (REPO_ROOT / "evaluation/virtual_formation_a/G4/" /
           "20261116T055008Z")
OLD_MANIFEST = OLD_RUN / "source_manifest.json"
SCHEMA = "virtual_rigid_a.g4.source_manifest.v1"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition, message):
    if not condition:
        raise SystemExit("HISTORICAL CHECK FAILED: " + message)


def main():
    manifest = json.loads(OLD_MANIFEST.read_text(encoding="utf-8"))
    require(manifest["schema"] == SCHEMA, "old manifest schema")
    require(manifest["run_id"] == OLD_RUN.name, "old manifest run id")
    excluded = set(manifest["excluded_self_referential"])
    checked = 0
    total_bytes = 0
    for relative, record in sorted(manifest["run_files"].items()):
        if relative in excluded:
            continue
        path = OLD_RUN / relative
        require(path.is_file(), "old run file missing: " + relative)
        require(path.stat().st_size == record["bytes"],
                "old run file size changed: " + relative)
        require(sha256(path) == record["sha256"],
                "old run file digest changed: " + relative)
        checked += 1
        total_bytes += record["bytes"]
    require(checked == manifest["run_file_count"],
            "old run file count")
    print(json.dumps({
        "status": "UNCHANGED",
        "old_run": str(OLD_RUN.relative_to(REPO_ROOT)),
        "old_manifest_sha256": sha256(OLD_MANIFEST),
        "old_manifest_recorded_run_files": manifest.get("run_file_count"),
        "checked_run_files": checked,
        "checked_bytes": total_bytes,
        "excluded_self_referential": sorted(excluded),
        "note": (
            "The previous G4 run is historical, read-only evidence.  Its "
            "own source manifest binds every checked file; all digests and "
            "sizes match."),
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()