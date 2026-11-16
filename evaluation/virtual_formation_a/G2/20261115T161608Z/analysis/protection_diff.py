"""Compare the G2 worktree with the Codex-accepted G1 protected snapshot.

This evidence assembler is read-only for every workspace file.  It recomputes
the protected tree from disk, verifies the accepted G1 evidence manifest, and
classifies every difference against the G2 whitelist.  It never claims Codex
acceptance for G2.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
G1_ACCEPTANCE = (REPO_ROOT / "evaluation/virtual_formation_a/G1" /
                 "codex_acceptance_rework_01/CODEX_ACCEPTANCE.json")
G1_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G1/20261115T152858Z"
G1_MANIFEST = G1_RUN / "source_manifest.json"
G1_PROTECTED = G1_RUN / "legacy/protected_after.json"
G0_SNAPSHOT = (REPO_ROOT / "evaluation/virtual_formation_a/G0" /
               "20261115T143321Z_rework/protected_before.json")
EXPECTED_G0_SHA = (
    "a89efe0887eb20aced4bad74bc8adf280079191b9c491436e2ae4e89c2506c9a")
EXPECTED_G0_TREE = (
    "ff58f2b5c6acfbc207327202f875e5e9a20734ef09ebb6070ee07edce6ebf091")
EXPECTED_G0_FILE_COUNT = 3120
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
EXPECTED_G1_MANIFEST_SHA = (
    "fa6ca4387bf57d35f76097f5e50c23715bf72256f186707b59f4e1ec4262718a")
EXPECTED_G1_TREE = (
    "910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519")
EXPECTED_G1_AFTER_SNAPSHOT_SHA = (
    "c9e4c874c3e7d0dc05a1850d74febd28ec7475a232b6f8f98d3e1c11d2892470")
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
ALLOWED_NEW = sorted([
    PACKAGE + "audit.py",
    PACKAGE + "optimizer.py",
    "tests/fixtures/virtual_a/existence_reference.json",
    "tests/fixtures/virtual_a/open_requests.json",
    "tests/test_virtual_a_audit.py",
    "tests/test_virtual_a_optimizer.py",
])
ALLOWED_CHANGED = sorted([
    PACKAGE + "contracts.py",
    "tools/virtual_a_benchmark.py",
])
EXCLUDED_BASENAMES = {"virtual_a_baseline.py"}
EXCLUDED_PARTS = {"__pycache__"}
EXCLUDED_SUFFIXES = {".pyc", ".pyo"}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def canonical_tree(entries):
    payload = {
        entry["path"]: {
            "type": entry["type"],
            "sha256": entry["sha256"],
            "size_bytes": entry["size_bytes"],
        }
        for entry in entries
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def current_entries(roots):
    rows = []
    for raw_root in roots:
        root = REPO_ROOT / raw_root
        if not root.exists():
            continue
        candidates = [root] if (root.is_file() or root.is_symlink()) else sorted(
            path for path in root.rglob("*")
            if path.is_file() or path.is_symlink())
        for path in candidates:
            relative = path.relative_to(REPO_ROOT).as_posix()
            parts = set(path.relative_to(REPO_ROOT).parts)
            if path.name in EXCLUDED_BASENAMES:
                continue
            if parts & EXCLUDED_PARTS:
                continue
            if path.suffix in EXCLUDED_SUFFIXES:
                continue
            if path.is_symlink():
                # Match the accepted G1 snapshot: the digest binds the link
                # target text and the logical size is zero.
                target_text = os.readlink(path)
                rows.append({
                    "path": relative,
                    "type": "symlink",
                    "sha256": hashlib.sha256(
                        os.fsencode(target_text)).hexdigest(),
                    "size_bytes": 0,
                    "symlink_target": target_text,
                })
            else:
                rows.append({
                    "path": relative,
                    "type": "file",
                    "sha256": sha256(path),
                    "size_bytes": path.stat().st_size,
                })
    return rows


def git(*args):
    completed = subprocess.run(
        ["git"] + list(args), cwd=str(REPO_ROOT), check=True,
        capture_output=True, text=True)
    return completed.stdout


def main():
    g1_snapshot = load(G1_PROTECTED)
    if sha256(G1_PROTECTED) != EXPECTED_G1_AFTER_SNAPSHOT_SHA:
        raise SystemExit("accepted G1 protected_after.json changed")
    if g1_snapshot["protected_tree_sha256"] != EXPECTED_G1_TREE:
        raise SystemExit("declared G1 protected tree changed")
    if canonical_tree(g1_snapshot["protected_files"]) != EXPECTED_G1_TREE:
        raise SystemExit("G1 protected snapshot is internally inconsistent")
    if sha256(G1_MANIFEST) != EXPECTED_G1_MANIFEST_SHA:
        raise SystemExit("accepted G1 source manifest changed")
    acceptance = load(G1_ACCEPTANCE)
    if acceptance.get("status") != "ACCEPTED":
        raise SystemExit("G1 acceptance status was not ACCEPTED")
    if acceptance.get("source_manifest_sha256") != EXPECTED_G1_MANIFEST_SHA:
        raise SystemExit("G1 acceptance does not bind the G1 manifest")
    if acceptance.get("next_stage_authorized") is not False:
        raise SystemExit("G1 acceptance unexpectedly authorizes G3")

    g0 = load(G0_SNAPSHOT)
    if sha256(G0_SNAPSHOT) != EXPECTED_G0_SHA:
        raise SystemExit("accepted G0 protected snapshot changed")
    if g0["protected_tree_sha256"] != EXPECTED_G0_TREE:
        raise SystemExit("declared G0 protected tree changed")
    if canonical_tree(g0["protected_files"]) != EXPECTED_G0_TREE:
        raise SystemExit("G0 protected snapshot is internally inconsistent")
    if g0["protected_file_count"] != EXPECTED_G0_FILE_COUNT:
        raise SystemExit("G0 protected file count changed")

    g1_manifest = load(G1_MANIFEST)
    excluded = set(g1_manifest.get("excluded_self_referential", []))
    checked_run_files = {}
    mismatches = []
    for relative, record in sorted(g1_manifest["run_files"].items()):
        path = G1_RUN / relative
        if relative in excluded:
            continue
        if not path.is_file():
            mismatches.append({"path": relative, "reason": "missing"})
            continue
        actual = sha256(path)
        if actual != record["sha256"]:
            mismatches.append({
                "path": relative, "reason": "sha256_changed",
                "recorded": record["sha256"], "actual": actual})
        checked_run_files[relative] = {
            "sha256": actual, "bytes": path.stat().st_size}
    if mismatches:
        raise SystemExit("accepted G1 run files changed: {}".format(
            mismatches[:3]))

    old_rows = {row["path"]: row for row in g1_snapshot["protected_files"]}
    new_rows_list = current_entries(g1_snapshot["protected_paths"])
    new_rows = {row["path"]: row for row in new_rows_list}
    added = sorted(set(new_rows) - set(old_rows))
    removed = sorted(set(old_rows) - set(new_rows))
    changed = sorted(
        path for path in set(old_rows) & set(new_rows)
        if (old_rows[path]["type"], old_rows[path]["sha256"],
            old_rows[path]["size_bytes"])
        != (new_rows[path]["type"], new_rows[path]["sha256"],
            new_rows[path]["size_bytes"]))
    unchanged = sorted(path for path in set(old_rows) & set(new_rows)
                       if path not in set(changed))
    old_unchanged_tree = canonical_tree([old_rows[path] for path in unchanged])
    new_unchanged_tree = canonical_tree([new_rows[path] for path in unchanged])
    if old_unchanged_tree != new_unchanged_tree:
        raise SystemExit("protected entries beyond the authorized changes "
                         "differ from the accepted G1 snapshot")
    full_tree = canonical_tree(new_rows_list)

    staged = git("diff", "--cached", "--binary")
    status = git("status", "--short")
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD"],
        cwd=str(REPO_ROOT), check=False).returncode
    if head != BASELINE_HEAD or branch != "feature/virtual-rigid-formation-a":
        raise SystemExit("unexpected HEAD/branch")
    if ancestor != 0:
        raise SystemExit("baseline is not an ancestor")
    if staged:
        raise SystemExit("staged changes are present")
    if added != ALLOWED_NEW:
        raise SystemExit("unexpected protected additions: {}".format(added))
    if changed != ALLOWED_CHANGED:
        raise SystemExit("unexpected protected changes: {}".format(changed))
    if removed:
        raise SystemExit("protected files were removed: {}".format(removed))

    result = {
        "schema": "virtual_rigid_a.g2.protection_diff.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "comparison":
            "current protected tree versus Codex-accepted G1 protected_after",
        "g1_acceptance": {
            "path": str(G1_ACCEPTANCE.relative_to(REPO_ROOT)),
            "sha256": sha256(G1_ACCEPTANCE),
            "status": acceptance["status"],
            "source_manifest_sha256":
                acceptance["source_manifest_sha256"],
            "next_stage_authorized": acceptance["next_stage_authorized"],
        },
        "g0_evidence": {
            "path": str(G0_SNAPSHOT.relative_to(REPO_ROOT)),
            "sha256": sha256(G0_SNAPSHOT),
            "protected_tree_sha256": g0["protected_tree_sha256"],
            "protected_file_count": g0["protected_file_count"],
            "unchanged_since_g1_acceptance": True,
        },
        "g1_evidence": {
            "run": str(G1_RUN.relative_to(REPO_ROOT)),
            "source_manifest_sha256": sha256(G1_MANIFEST),
            "protected_after_sha256": sha256(G1_PROTECTED),
            "protected_tree_sha256": g1_snapshot["protected_tree_sha256"],
            "protected_file_count": g1_snapshot["protected_file_count"],
            "run_files_checked": len(checked_run_files),
            "run_files_excluded_self_referential": sorted(excluded),
            "run_files_all_match": True,
        },
        "protected_roots": g1_snapshot["protected_paths"],
        "current": {
            "g1_file_count": len(old_rows),
            "current_file_count": len(new_rows_list),
            "common_unchanged_count": len(unchanged),
            "unchanged_common_tree_sha256_g1_snapshot": old_unchanged_tree,
            "unchanged_common_tree_sha256_current": new_unchanged_tree,
            "unchanged_common_tree_matches": (
                old_unchanged_tree == new_unchanged_tree),
            "full_protected_tree_sha256": full_tree,
            "additions_count": len(added),
            "changed_count": len(changed),
            "removed_count": len(removed),
        },
        "added_vs_g1_accepted": [
            {"path": path, "sha256": new_rows[path]["sha256"],
             "size_bytes": new_rows[path]["size_bytes"]} for path in added],
        "removed_vs_g1_accepted": removed,
        "changed_vs_g1_accepted": [
            {"path": path,
             "g1_sha256": old_rows[path]["sha256"],
             "g2_sha256": new_rows[path]["sha256"],
             "g1_size_bytes": old_rows[path]["size_bytes"],
             "g2_size_bytes": new_rows[path]["size_bytes"]}
            for path in changed],
        "authorization": {
            "allowed_new_files": ALLOWED_NEW,
            "allowed_tracked_changes": ALLOWED_CHANGED,
            "additions_exactly_whitelisted": added == ALLOWED_NEW,
            "changes_exactly_whitelisted": changed == ALLOWED_CHANGED,
            "no_protected_removals": removed == [],
            "only_authorized_changes": (
                added == ALLOWED_NEW and removed == []
                and changed == ALLOWED_CHANGED),
            "note": (
                "G1 tests, initial-state/C2/vehicle limits, package "
                "__init__, geometry, bounds, setup.py, old FR06 source and "
                "historical evidence are unchanged. Path authorization is "
                "not Codex acceptance."),
        },
        "git": {
            "head": head,
            "branch": branch,
            "baseline_head": BASELINE_HEAD,
            "baseline_is_ancestor": True,
            "staged_changes_present": False,
            "status_short": status.splitlines(),
            "committed": "NO",
            "pushed": "NO",
        },
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }
    target = RUN / "analysis" / "protection_diff_vs_g1_accepted.json"
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "added": added,
        "changed": changed,
        "removed": removed,
        "unchanged_common_tree": old_unchanged_tree,
        "full_protected_tree": full_tree,
        "g1_tree": g1_snapshot["protected_tree_sha256"],
        "g1_run_files_checked": len(checked_run_files),
        "only_authorized": result["authorization"]["only_authorized_changes"],
        "sha256": sha256(target),
    }, indent=2))


if __name__ == "__main__":
    main()