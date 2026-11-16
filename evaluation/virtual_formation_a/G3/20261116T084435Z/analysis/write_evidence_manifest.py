"""Bind the G3 non-finite fix evidence and its source files."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[5]
SCHEMA = "virtual_rigid_a.g3.nonfinite_fix.manifest.v1"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
SOURCE_FILES = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/audit.py",
    "tests/test_virtual_a_audit.py",
)
BOUND_FILES = (
    "docs/virtual_formation_a/CLAUDE_G3_NONFINITE_FIX_TASK.md",
    "evaluation/virtual_formation_a/G3/codex_acceptance_rework_01/CODEX_ACCEPTANCE.json",
    "evaluation/virtual_formation_a/G4/codex_acceptance_rework_01/CODEX_ACCEPTANCE.json",
    "evaluation/virtual_formation_a/G3/20261116T031558Z/source_manifest.json",
    "evaluation/virtual_formation_a/G4/20261116T081458Z/source_manifest.json",
    "evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json",
    "evaluation/virtual_formation_a/G4/20261116T081458Z/benchmark/results.json",
)
EXCLUSIONS = (
    "source_manifest.json",
    "logs/source_manifest_generation.log",
    "logs/source_manifest_verification.log",
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def record(path):
    path = Path(path)
    return {"bytes": path.stat().st_size, "sha256": sha256(path)}


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(ROOT), check=True,
        capture_output=True, text=True).stdout


def main():
    run_files = {}
    for path in sorted(RUN.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(RUN).as_posix()
        if relative in EXCLUSIONS:
            continue
        run_files[relative] = record(path)
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    if head != BASELINE_HEAD or branch != "feature/virtual-rigid-formation-a":
        raise SystemExit("unexpected HEAD or branch")
    if staged:
        raise SystemExit("staged changes present")
    results = json.loads((RUN / "results.json").read_text(encoding="utf-8"))
    if results["status"] != "PASS_PENDING_CODEX_RECHECK":
        raise SystemExit("results status changed")
    manifest = {
        "schema": SCHEMA,
        "stage": "G3_NONFINITE_FIX",
        "run_id": RUN.name,
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "repo_root": str(ROOT),
        "baseline_head": BASELINE_HEAD,
        "excluded_self_referential": list(EXCLUSIONS),
        "source_files": {
            relative: record(ROOT / relative)
            for relative in SOURCE_FILES},
        "bound_files": {
            relative: record(ROOT / relative)
            for relative in BOUND_FILES},
        "run_files": run_files,
        "run_file_count": len(run_files),
        "run_total_bytes": sum(
            value["bytes"] for value in run_files.values()),
        "results_sha256": sha256(RUN / "results.json"),
        "git": {
            "head": head,
            "branch": branch,
            "staged_bytes": len(staged.encode("utf-8")),
            "committed": "NO",
            "pushed": "NO",
            "branch_created": "NO",
        },
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }
    target = RUN / "source_manifest.json"
    target.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "path": str(target.relative_to(ROOT)),
        "source_file_count": len(SOURCE_FILES),
        "bound_file_count": len(BOUND_FILES),
        "run_file_count": len(run_files),
        "run_total_bytes": manifest["run_total_bytes"],
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()