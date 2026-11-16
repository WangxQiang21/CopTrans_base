"""Compare the G4 worktree with the Codex-accepted G3 protected tree.

The accepted G3 run did not save a full protected snapshot.  Its protected tree
is reconstructed from the accepted G1 snapshot plus the authorized G2 and G3
additions/changes recorded in their protection diffs, and the reconstruction is
checked against the tree digest bound by the G3 source manifest and the Codex
G3 acceptance.  This assembler is read-only for every workspace file and never
claims Codex acceptance for G4.
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
G3_ACCEPTANCE = (REPO_ROOT / "evaluation/virtual_formation_a/G3" /
                 "codex_acceptance_rework_01/CODEX_ACCEPTANCE.json")
G3_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G3/20261116T031558Z"
G3_MANIFEST = G3_RUN / "source_manifest.json"
G3_PROTECTION = G3_RUN / "analysis/protection_diff_vs_g2_accepted.json"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
EXPECTED_G1_TREE = (
    "910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519")
EXPECTED_G1_AFTER_SHA = (
    "c9e4c874c3e7d0dc05a1850d74febd28ec7475a232b6f8f98d3e1c11d2892470")
EXPECTED_G2_MANIFEST_SHA = (
    "852cb44615321ed69613f6dfbe6ae6b2b79da5a17563c38537e899209dc90886")
EXPECTED_G2_TREE = (
    "fa1b0596f7b9ecef616512785ab65ef2b93884dfde3a9c2a9794a9f7ac222f67")
EXPECTED_G3_MANIFEST_SHA = (
    "bad6f55441bd34ca8d164cc954170336b56df398fcb0912dae10a4588279ceea")
EXPECTED_G3_TREE = (
    "9bee7d7a483418877277fe63c83986363f7ffc114f03db6ff265927092d89fba")
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
SCENE_NAMES = ("S1", "S1_mirror", "S2", "S2_mirror", "S3", "S3_mirror")
SYNTHETIC_NAMES = (
    "arc_corner", "boundary_goal", "blocked_start", "corridor",
    "door_circumcircle", "thin_barrier", "unknown_gray")
ALLOWED_NEW = sorted(
    [PACKAGE + "collision.py", PACKAGE + "pipeline.py"] +
    ["tests/fixtures/virtual_a/maps/scenes/{}.{}".format(
        name, extension)
     for name in SCENE_NAMES for extension in ("pgm", "yaml")] +
    ["tests/fixtures/virtual_a/maps/synthetic/{}/map.{}".format(
        name, extension)
     for name in SYNTHETIC_NAMES for extension in ("pgm", "yaml")] +
    ["tests/fixtures/virtual_a/scenes/{}.request.json".format(name)
     for name in SCENE_NAMES] +
    ["tests/test_virtual_a_collision.py",
     "tests/test_virtual_a_pipeline.py",
     "tests/virtual_a_g4_support.py",
     "tools/virtual_a_scenes.py"])
ALLOWED_CHANGED = sorted([
    PACKAGE + "audit.py",
    PACKAGE + "contracts.py",
    PACKAGE + "optimizer.py",
    PACKAGE + "trajectory.py",
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
        candidates = [root] if (root.is_file() or root.is_symlink()) else \
            sorted(path for path in root.rglob("*")
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


def rows_from_snapshot(snapshot):
    return {row["path"]: dict(row) for row in snapshot["protected_files"]}


def apply_diff(rows, diff, sha_key, size_key):
    for row in diff:
        rows[row["path"]] = {
            "path": row["path"], "type": "file",
            "sha256": row[sha_key], "size_bytes": row[size_key]}
    return rows


def verify_g2(g2_manifest, g2_protection):
    acceptance = load(G2_ACCEPTANCE)
    if acceptance.get("status") != "ACCEPTED":
        raise SystemExit("G2 acceptance status was not ACCEPTED")
    if acceptance.get("source_manifest_sha256") != EXPECTED_G2_MANIFEST_SHA:
        raise SystemExit("G2 acceptance does not bind the G2 manifest")
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
    return acceptance


def verify_g3(g3_manifest, g3_protection):
    acceptance = load(G3_ACCEPTANCE)
    if acceptance.get("status") != "ACCEPTED":
        raise SystemExit("G3 acceptance status was not ACCEPTED")
    if acceptance.get("source_manifest_sha256") != EXPECTED_G3_MANIFEST_SHA:
        raise SystemExit("G3 acceptance does not bind the G3 manifest")
    if acceptance.get("next_stage_authorized") is not False:
        raise SystemExit("G3 acceptance unexpectedly authorizes G4")
    if sha256(G3_MANIFEST) != EXPECTED_G3_MANIFEST_SHA:
        raise SystemExit("accepted G3 source manifest changed")
    if g3_manifest["protected"][
            "current_full_protected_tree_sha256"] != EXPECTED_G3_TREE:
        raise SystemExit("G3 manifest protected tree changed")
    if g3_protection["current"][
            "full_protected_tree_sha256"] != EXPECTED_G3_TREE:
        raise SystemExit("G3 protection-tree digest changed")
    if g3_protection["authorization"][
            "only_authorized_changes"] is not True:
        raise SystemExit("G3 protection diff was not exactly authorized")
    if g3_protection["removed_vs_g2_accepted"]:
        raise SystemExit("G3 recorded protected removals")
    excluded = set(g3_manifest.get("excluded_self_referential", []))
    mismatches = []
    checked = {}
    for relative, record in sorted(g3_manifest["run_files"].items()):
        if relative in excluded:
            continue
        path = G3_RUN / relative
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
        raise SystemExit("accepted G3 run files changed: {}".format(
            mismatches[:3]))
    for relative, record in acceptance["source_sha256"].items():
        source = g3_manifest["source_files"].get(relative)
        if source is None or source["sha256"] != record:
            raise SystemExit(
                "G3 acceptance source hash is not bound by its manifest: " +
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
    verify_g2(g2_manifest, g2_protection)

    g3_manifest = load(G3_MANIFEST)
    g3_protection = load(G3_PROTECTION)
    g3_acceptance, g3_excluded, g3_run_checked = verify_g3(
        g3_manifest, g3_protection)

    # Reconstruct the accepted G2 protected tree from G1 plus the recorded G2
    # additions and changes; the tree digest must equal the accepted binding.
    g2_rows = rows_from_snapshot(g1_snapshot)
    apply_diff(g2_rows, g2_protection["added_vs_g1_accepted"],
               "sha256", "size_bytes")
    apply_diff(g2_rows, g2_protection["changed_vs_g1_accepted"],
               "g2_sha256", "g2_size_bytes")
    if canonical_tree(g2_rows.values()) != EXPECTED_G2_TREE:
        raise SystemExit("G2 protected-tree reconstruction does not match "
                         "the accepted digest")

    # Reconstruct the accepted G3 protected tree from G2 plus the recorded G3
    # additions and changes; the tree digest must equal the accepted binding.
    g3_rows = {path: dict(row) for path, row in g2_rows.items()}
    apply_diff(g3_rows, g3_protection["added_vs_g2_accepted"],
               "sha256", "size_bytes")
    apply_diff(g3_rows, g3_protection["changed_vs_g2_accepted"],
               "g3_sha256", "g3_size_bytes")
    if sorted(g3_rows) != sorted(
            set(g2_rows) |
            set(row["path"] for row in
                g3_protection["added_vs_g2_accepted"])):
        raise SystemExit("reconstructed G3 path set is inconsistent")
    reconstructed_tree = canonical_tree(g3_rows.values())
    if reconstructed_tree != EXPECTED_G3_TREE:
        raise SystemExit("G3 protected-tree reconstruction does not match "
                         "the accepted digest")
    for row in g3_protection["added_vs_g2_accepted"]:
        source = g3_manifest["source_files"].get(row["path"])
        if source is None or source["sha256"] != row["sha256"]:
            raise SystemExit("G3 added protected file is not bound by the "
                             "accepted manifest: " + row["path"])
    for row in g3_protection["changed_vs_g2_accepted"]:
        source = g3_manifest["source_files"].get(row["path"])
        if source is None or source["sha256"] != row["g3_sha256"]:
            raise SystemExit("G3 changed protected file is not bound by the "
                             "accepted manifest: " + row["path"])

    old_rows = g3_rows
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
                         "differ from the reconstructed accepted G3 tree")
    full_tree = canonical_tree(new_rows_list)

    staged = git("diff", "--cached", "--binary")
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
        "schema": "virtual_rigid_a.g4.protection_diff.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "comparison": (
            "current protected tree versus Codex-accepted G3 protected tree "
            "reconstructed from the accepted G1 snapshot plus the authorized "
            "G2 and G3 additions/changes"),
        "g3_acceptance": {
            "path": str(G3_ACCEPTANCE.relative_to(REPO_ROOT)),
            "sha256": sha256(G3_ACCEPTANCE),
            "status": g3_acceptance["status"],
            "source_manifest_sha256":
                g3_acceptance["source_manifest_sha256"],
            "next_stage_authorized":
                g3_acceptance["next_stage_authorized"],
        },
        "g3_evidence": {
            "run": str(G3_RUN.relative_to(REPO_ROOT)),
            "source_manifest_sha256": sha256(G3_MANIFEST),
            "protection_diff_sha256": sha256(G3_PROTECTION),
            "protected_tree_sha256": reconstructed_tree,
            "protected_file_count": len(g3_rows),
            "run_files_checked": len(g3_run_checked),
            "run_files_excluded_self_referential": sorted(g3_excluded),
            "run_files_all_match": True,
            "reconstruction_from_g1_plus_g2_and_g3_changes": True,
        },
        "g2_evidence": {
            "run": str(G2_RUN.relative_to(REPO_ROOT)),
            "source_manifest_sha256": sha256(G2_MANIFEST),
            "protection_diff_sha256": sha256(G2_PROTECTION),
            "protected_tree_sha256": EXPECTED_G2_TREE,
        },
        "g1_evidence": {
            "run": str(G1_RUN.relative_to(REPO_ROOT)),
            "protected_after_sha256": sha256(G1_PROTECTED),
            "protected_tree_sha256": g1_snapshot["protected_tree_sha256"],
            "protected_file_count": g1_snapshot["protected_file_count"],
        },
        "protected_roots": g1_snapshot["protected_paths"],
        "current": {
            "g3_file_count": len(old_rows),
            "current_file_count": len(new_rows_list),
            "common_unchanged_count": len(unchanged),
            "unchanged_common_tree_sha256_g3_reconstructed":
                old_unchanged_tree,
            "unchanged_common_tree_sha256_current": new_unchanged_tree,
            "unchanged_common_tree_matches": (
                old_unchanged_tree == new_unchanged_tree),
            "full_protected_tree_sha256": full_tree,
            "additions_count": len(added),
            "changed_count": len(changed),
            "removed_count": len(removed),
        },
        "added_vs_g3_accepted": [
            {"path": path, "sha256": new_rows[path]["sha256"],
             "size_bytes": new_rows[path]["size_bytes"]} for path in added],
        "removed_vs_g3_accepted": removed,
        "changed_vs_g3_accepted": [
            {"path": path,
             "g3_sha256": old_rows[path]["sha256"],
             "g4_sha256": new_rows[path]["sha256"],
             "g3_size_bytes": old_rows[path]["size_bytes"],
             "g4_size_bytes": new_rows[path]["size_bytes"]}
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
                "G1/G2/G3 source and tests outside the G4 whitelist, old FR06 "
                "source, old maps and the old collision engine are "
                "unchanged. Path authorization is not Codex acceptance."),
        },
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "git": {
            "head": head,
            "branch": branch,
            "baseline_is_ancestor": ancestor == 0,
            "staged_changes": bool(staged),
        },
    }
    target = RUN / "analysis/protection_diff_vs_g3_accepted.json"
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "path": str(target.relative_to(REPO_ROOT)),
        "current_file_count": len(new_rows_list),
        "additions": len(added),
        "changed": len(changed),
        "removed": len(removed),
        "full_protected_tree_sha256": full_tree,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()