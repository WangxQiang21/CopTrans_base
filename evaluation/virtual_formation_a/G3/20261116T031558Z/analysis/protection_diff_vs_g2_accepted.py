"""Compare the G3 worktree with the Codex-accepted G2 protected tree.

The G2 run did not save a full protected snapshot.  Its accepted protected tree
is reconstructed from the accepted G1 snapshot plus the six G2 additions and
two G2 changes recorded in the G2 protection diff, and the reconstruction is
checked against the tree digest bound by both the G2 source manifest and the
Codex acceptance.  This assembler is read-only for every workspace file and
never claims Codex acceptance for G3.
"""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
G1_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G1/20261115T152858Z"
G1_PROTECTED = G1_RUN / "legacy/protected_after.json"
G2_ACCEPTANCE = (REPO_ROOT / "evaluation/virtual_formation_a/G2" /
                 "codex_acceptance_rework_03/CODEX_ACCEPTANCE.json")
G2_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G2/20261115T174526Z"
G2_MANIFEST = G2_RUN / "source_manifest.json"
G2_PROTECTION = G2_RUN / "analysis/protection_diff_vs_g1_accepted.json"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
EXPECTED_G1_TREE = (
    "910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519")
EXPECTED_G1_AFTER_SHA = (
    "c9e4c874c3e7d0dc05a1850d74febd28ec7475a232b6f8f98d3e1c11d2892470")
EXPECTED_G2_MANIFEST_SHA = (
    "852cb44615321ed69613f6dfbe6ae6b2b79da5a17563c38537e899209dc90886")
EXPECTED_G2_TREE = (
    "fa1b0596f7b9ecef616512785ab65ef2b93884dfde3a9c2a9794a9f7ac222f67")
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
ALLOWED_NEW = sorted([
    PACKAGE + "timing.py",
    PACKAGE + "trajectory.py",
    "tests/test_virtual_a_timing.py",
    "tests/test_virtual_a_trajectory.py",
])
ALLOWED_CHANGED = sorted([
    PACKAGE + "audit.py",
    PACKAGE + "bounds.py",
    PACKAGE + "contracts.py",
    "tests/test_virtual_a_audit.py",
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


def verify_accepted_g2(g2_manifest, g2_protection):
    acceptance = load(G2_ACCEPTANCE)
    if acceptance.get("status") != "ACCEPTED":
        raise SystemExit("G2 acceptance status was not ACCEPTED")
    if acceptance.get("source_manifest_sha256") != EXPECTED_G2_MANIFEST_SHA:
        raise SystemExit("G2 acceptance does not bind the G2 manifest")
    if acceptance.get("next_stage_authorized") is not False:
        raise SystemExit("G2 acceptance unexpectedly authorizes G3")
    if sha256(G2_MANIFEST) != EXPECTED_G2_MANIFEST_SHA:
        raise SystemExit("accepted G2 source manifest changed")
    if g2_manifest["protected"][
            "current_full_protected_tree_sha256"] != EXPECTED_G2_TREE:
        raise SystemExit("G2 manifest protected tree changed")
    if g2_protection["current"][
            "full_protected_tree_sha256"] != EXPECTED_G2_TREE:
        raise SystemExit("G2 protection-tree digest changed")
    if g2_protection["authorization"][
            "only_authorized_changes"] is not True:
        raise SystemExit("G2 protection diff was not exactly authorized")
    if g2_protection["removed_vs_g1_accepted"]:
        raise SystemExit("G2 recorded protected removals")
    excluded = set(g2_manifest.get("excluded_self_referential", []))
    checked = {}
    mismatches = []
    for relative, record in sorted(g2_manifest["run_files"].items()):
        if relative in excluded:
            continue
        path = G2_RUN / relative
        if not path.is_file():
            mismatches.append({"path": relative, "reason": "missing"})
            continue
        actual = sha256(path)
        if actual != record["sha256"]:
            mismatches.append({
                "path": relative, "reason": "sha256_changed",
                "recorded": record["sha256"], "actual": actual})
        checked[relative] = {
            "sha256": actual, "bytes": path.stat().st_size}
    if mismatches:
        raise SystemExit("accepted G2 run files changed: {}".format(
            mismatches[:3]))
    for relative, record in acceptance["source_sha256"].items():
        source = g2_manifest["source_files"].get(relative)
        if source is None or source["sha256"] != record:
            raise SystemExit(
                "G2 acceptance source hash is not bound by its manifest: " +
                relative)
    return acceptance, excluded, checked


def main():
    g1_snapshot = load(G1_PROTECTED)
    if sha256(G1_PROTECTED) != EXPECTED_G1_AFTER_SHA:
        raise SystemExit("accepted G1 protected_after.json changed")
    if g1_snapshot["protected_tree_sha256"] != EXPECTED_G1_TREE:
        raise SystemExit("declared G1 protected tree changed")
    if canonical_tree(g1_snapshot["protected_files"]) != EXPECTED_G1_TREE:
        raise SystemExit("G1 protected snapshot is internally inconsistent")

    g2_manifest = load(G2_MANIFEST)
    g2_protection = load(G2_PROTECTION)
    acceptance, g2_excluded, g2_run_checked = verify_accepted_g2(
        g2_manifest, g2_protection)

    # Reconstruct the accepted G2 protected tree from G1 plus the recorded G2
    # additions and changes; the tree digest must equal the accepted binding.
    g2_rows = {row["path"]: dict(row) for row in
               g1_snapshot["protected_files"]}
    for row in g2_protection["added_vs_g1_accepted"]:
        g2_rows[row["path"]] = {
            "path": row["path"], "type": "file",
            "sha256": row["sha256"], "size_bytes": row["size_bytes"]}
    for row in g2_protection["changed_vs_g1_accepted"]:
        old = g2_rows[row["path"]]
        g2_rows[row["path"]] = {
            "path": row["path"], "type": old["type"],
            "sha256": row["g2_sha256"],
            "size_bytes": row["g2_size_bytes"]}
    if sorted(g2_rows) != sorted(
            set(row["path"] for row in g1_snapshot["protected_files"]) |
            set(row["path"] for row in
                g2_protection["added_vs_g1_accepted"])):
        raise SystemExit("reconstructed G2 path set is inconsistent")
    reconstructed_tree = canonical_tree(g2_rows.values())
    if reconstructed_tree != EXPECTED_G2_TREE:
        raise SystemExit("G2 protected-tree reconstruction does not match "
                         "the accepted digest")
    for row in g2_protection["added_vs_g1_accepted"]:
        source = g2_manifest["source_files"].get(row["path"])
        if source is None or source["sha256"] != row["sha256"]:
            raise SystemExit("G2 added protected file is not bound by the "
                             "accepted manifest: " + row["path"])
    for row in g2_protection["changed_vs_g1_accepted"]:
        source = g2_manifest["source_files"].get(row["path"])
        if source is None or source["sha256"] != row["g2_sha256"]:
            raise SystemExit("G2 changed protected file is not bound by the "
                             "accepted manifest: " + row["path"])

    old_rows = g2_rows
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
                         "differ from the reconstructed accepted G2 tree")
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
        "schema": "virtual_rigid_a.g3.protection_diff.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "comparison": (
            "current protected tree versus Codex-accepted G2 protected tree "
            "reconstructed from the accepted G1 snapshot plus the G2 "
            "authorized additions/changes"),
        "g2_acceptance": {
            "path": str(G2_ACCEPTANCE.relative_to(REPO_ROOT)),
            "sha256": sha256(G2_ACCEPTANCE),
            "status": acceptance["status"],
            "source_manifest_sha256":
                acceptance["source_manifest_sha256"],
            "next_stage_authorized": acceptance["next_stage_authorized"],
        },
        "g2_evidence": {
            "run": str(G2_RUN.relative_to(REPO_ROOT)),
            "source_manifest_sha256": sha256(G2_MANIFEST),
            "protection_diff_sha256": sha256(G2_PROTECTION),
            "protected_tree_sha256": reconstructed_tree,
            "protected_file_count": len(g2_rows),
            "run_files_checked": len(g2_run_checked),
            "run_files_excluded_self_referential": sorted(g2_excluded),
            "run_files_all_match": True,
            "reconstruction_from_g1_plus_g2_changes": True,
        },
        "g1_evidence": {
            "run": str(G1_RUN.relative_to(REPO_ROOT)),
            "protected_after_sha256": sha256(G1_PROTECTED),
            "protected_tree_sha256": g1_snapshot["protected_tree_sha256"],
            "protected_file_count": g1_snapshot["protected_file_count"],
        },
        "protected_roots": g1_snapshot["protected_paths"],
        "current": {
            "g2_file_count": len(old_rows),
            "current_file_count": len(new_rows_list),
            "common_unchanged_count": len(unchanged),
            "unchanged_common_tree_sha256_g2_reconstructed":
                old_unchanged_tree,
            "unchanged_common_tree_sha256_current": new_unchanged_tree,
            "unchanged_common_tree_matches": (
                old_unchanged_tree == new_unchanged_tree),
            "full_protected_tree_sha256": full_tree,
            "additions_count": len(added),
            "changed_count": len(changed),
            "removed_count": len(removed),
        },
        "added_vs_g2_accepted": [
            {"path": path, "sha256": new_rows[path]["sha256"],
             "size_bytes": new_rows[path]["size_bytes"]} for path in added],
        "removed_vs_g2_accepted": removed,
        "changed_vs_g2_accepted": [
            {"path": path,
             "g2_sha256": old_rows[path]["sha256"],
             "g3_sha256": new_rows[path]["sha256"],
             "g2_size_bytes": old_rows[path]["size_bytes"],
             "g3_size_bytes": new_rows[path]["size_bytes"]}
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
                "G1/G2 source and tests outside the G3 whitelist, geometry, "
                "optimizer, fixtures, setup.py, old FR06 source, ROS/YAML/"
                "maps and historical evidence are unchanged. Path "
                "authorization is not Codex acceptance."),
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
    target = RUN / "analysis" / "protection_diff_vs_g2_accepted.json"
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "added": added,
        "changed": changed,
        "removed": removed,
        "g2_reconstructed_tree": reconstructed_tree,
        "g3_full_protected_tree": full_tree,
        "g2_run_files_checked": len(g2_run_checked),
        "only_authorized": result["authorization"]["only_authorized_changes"],
        "sha256": sha256(target),
    }, indent=2))


if __name__ == "__main__":
    main()