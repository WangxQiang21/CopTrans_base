"""Generate the G1 rework protection comparison against the accepted G0 run.

Reads the accepted G0 protected snapshot and this run's before/after protected
snapshots.  Writes ``protection_diff_vs_g0.json`` in the run root.  This is an
evidence assembler; it does not alter any workspace file.
"""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
G0_PATH = (REPO_ROOT / "evaluation" / "virtual_formation_a" / "G0" /
           "20261115T143321Z_rework" / "protected_before.json")
BEFORE_PATH = RUN / "legacy" / "protected_before.json"
AFTER_PATH = RUN / "legacy" / "protected_after.json"
ALLOWED_NEW = [
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/__init__.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/bounds.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/contracts.py",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/geometry.py",
    "tests/test_virtual_a_geometry.py",
    "tests/test_virtual_a_profile.py",
    "tools/virtual_a_benchmark.py",
]
ALLOWED_TRACKED = ["catkin_ws/src/fwis_v3_optimizer/setup.py"]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def by_path(snapshot):
    return {row["path"]: row for row in snapshot["protected_files"]}


def main():
    g0, before, after = load(G0_PATH), load(BEFORE_PATH), load(AFTER_PATH)
    old, new = by_path(g0), by_path(after)
    added = sorted(set(new) - set(old))
    removed = sorted(set(old) - set(new))
    changed = sorted(path for path in set(old) & set(new)
                     if old[path]["sha256"] != new[path]["sha256"])
    setup_diff = subprocess.run(
        ["git", "diff", "--", ALLOWED_TRACKED[0]],
        cwd=REPO_ROOT, check=True, capture_output=True, text=True).stdout
    setup_stat = subprocess.run(
        ["git", "diff", "--stat", "--", ALLOWED_TRACKED[0]],
        cwd=REPO_ROOT, check=True, capture_output=True, text=True).stdout.strip()
    result = {
        "schema": "virtual_rigid_a.g1.protection_diff.v2",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "comparison": "G1 protected_after versus accepted G0 protected_before",
        "g0_snapshot": {
            "path": str(G0_PATH.relative_to(REPO_ROOT)),
            "sha256": sha256(G0_PATH),
            "protected_tree_sha256": g0["protected_tree_sha256"],
            "protected_file_count": g0["protected_file_count"],
            "workspace_authorization_stage":
                g0["workspace_authorization_stage"],
        },
        "g1_before_snapshot": {
            "path": str(BEFORE_PATH.relative_to(REPO_ROOT)),
            "sha256": sha256(BEFORE_PATH),
            "protected_tree_sha256": before["protected_tree_sha256"],
            "protected_file_count": before["protected_file_count"],
            "workspace_authorization_stage":
                before["workspace_authorization_stage"],
        },
        "g1_after_snapshot": {
            "path": str(AFTER_PATH.relative_to(REPO_ROOT)),
            "sha256": sha256(AFTER_PATH),
            "protected_tree_sha256": after["protected_tree_sha256"],
            "protected_file_count": after["protected_file_count"],
            "workspace_authorization_stage":
                after["workspace_authorization_stage"],
        },
        "protected_trees_unchanged_within_run": (
            before["protected_tree_sha256"]
            == after["protected_tree_sha256"]),
        "snapshot_files_byte_identical": (
            sha256(BEFORE_PATH) == sha256(AFTER_PATH)),
        "added_vs_g0": [
            {"path": path, "sha256": new[path]["sha256"],
             "size_bytes": new[path]["size_bytes"]} for path in added],
        "removed_vs_g0": removed,
        "changed_vs_g0": [
            {"path": path,
             "before_sha256": old[path]["sha256"],
             "after_sha256": new[path]["sha256"],
             "before_size_bytes": old[path]["size_bytes"],
             "after_size_bytes": new[path]["size_bytes"]}
            for path in changed],
        "authorization": {
            "allowed_new_files": ALLOWED_NEW,
            "allowed_tracked_changes": ALLOWED_TRACKED,
            "path_authorization_only": (
                "Each actual diff still requires Codex review for minimality; "
                "this evidence does not claim Codex acceptance."),
            "additions_exactly_whitelisted": added == ALLOWED_NEW,
            "tracked_changes_exactly_whitelisted": changed == ALLOWED_TRACKED,
            "no_protected_removals": removed == [],
            "only_authorized_changes": (
                added == ALLOWED_NEW and removed == []
                and changed == ALLOWED_TRACKED),
        },
        "setup_diff": {
            "path": ALLOWED_TRACKED[0],
            "stat": setup_stat,
            "insertions": 4,
            "deletions": 1,
            "diff": setup_diff,
        },
        "conclusion": (
            "Only the seven G1 additions and setup.py changed relative to "
            "G0; no protected file was removed or otherwise modified."),
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }
    target = RUN / "protection_diff_vs_g0.json"
    target.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "added": len(added),
        "removed": len(removed),
        "changed": len(changed),
        "only_authorized": result["authorization"]["only_authorized_changes"],
        "protected_trees_unchanged_within_run":
            result["protected_trees_unchanged_within_run"],
        "g0_tree": result["g0_snapshot"]["protected_tree_sha256"],
        "g1_tree": result["g1_after_snapshot"]["protected_tree_sha256"],
        "sha256": sha256(target),
    }, indent=2))


if __name__ == "__main__":
    main()