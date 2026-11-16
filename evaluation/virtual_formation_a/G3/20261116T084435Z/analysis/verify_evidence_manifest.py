"""Verify the G3 non-finite fix evidence manifest without writing into it."""
import hashlib
import json
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[5]
SCHEMA = "virtual_rigid_a.g3.nonfinite_fix.manifest.v1"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition, label):
    if not condition:
        raise SystemExit("MANIFEST CHECK FAILED: " + label)


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(ROOT), check=True,
        capture_output=True, text=True).stdout


def main():
    path = RUN / "source_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    require(manifest["schema"] == SCHEMA, "schema")
    require(manifest["run_id"] == RUN.name, "run id")
    excluded = set(manifest["excluded_self_referential"])
    checked = 0
    checked_bytes = 0
    for relative, record in sorted(manifest["run_files"].items()):
        if relative in excluded:
            continue
        target = RUN / relative
        require(target.is_file(), "run file missing: " + relative)
        require(target.stat().st_size == record["bytes"],
                "run file size changed: " + relative)
        require(sha256(target) == record["sha256"],
                "run file digest changed: " + relative)
        checked += 1
        checked_bytes += record["bytes"]
    require(checked == manifest["run_file_count"], "run file count")
    require(checked_bytes == manifest["run_total_bytes"], "run bytes")
    for relative, record in sorted(manifest["source_files"].items()):
        target = ROOT / relative
        require(target.is_file() and sha256(target) == record["sha256"],
                "source file changed: " + relative)
    for relative, record in sorted(manifest["bound_files"].items()):
        target = ROOT / relative
        require(target.is_file() and sha256(target) == record["sha256"],
                "bound file changed: " + relative)
    require(sha256(RUN / "results.json") == manifest["results_sha256"],
            "results digest")
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    require(head == BASELINE_HEAD, "HEAD")
    require(branch == "feature/virtual-rigid-formation-a", "branch")
    require(staged == "", "staged changes")
    print(json.dumps({
        "status": "VERIFIED",
        "run_id": RUN.name,
        "source_manifest_sha256": sha256(path),
        "source_file_count": len(manifest["source_files"]),
        "bound_file_count": len(manifest["bound_files"]),
        "run_file_count": checked,
        "run_total_bytes": checked_bytes,
        "git_head": head,
        "git_branch": branch,
        "excluded_self_referential": sorted(excluded),
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()