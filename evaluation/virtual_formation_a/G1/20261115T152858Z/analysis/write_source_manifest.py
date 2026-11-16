"""Write the G1 rework source-and-evidence manifest.

The manifest binds every file in this evidence run except three explicitly
self-referential artifacts (the manifest itself, this generator's console log,
and the verifier's console log).  It also binds the source inventory and the
old evidence trees so that a later verifier can detect any silent change.
"""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
RUN_ID = RUN.name
SCHEMA = "virtual_rigid_a.g1.rework.source_manifest.v2"
STAGE = "G1_REWORK_01"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
REWORK_DOC = "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md"
OLD_SUBMITTED = (REPO_ROOT / "evaluation" / "virtual_formation_a" /
                 "G1" / "20261115T150639Z")
OLD_PRELIMINARY = (REPO_ROOT / "evaluation" / "virtual_formation_a" /
                   "G1" / "20261115T150114Z")
G0_PATH = (REPO_ROOT / "evaluation" / "virtual_formation_a" / "G0" /
           "20261115T143321Z_rework" / "protected_before.json")
EXCLUSIONS = [
    "logs/source_manifest_verification.log",
    "logs/source_manifest_generation.log",
    "source_manifest.json",
]
EXPECTED_OLD = {
    "submitted": {
        "manifest_sha256":
            "c40d3ac534aac957cc2206979ab85ef26f073bc6ca89fe5baf882d7cf124bc14",
        "tree_sha256":
            "d14cc53ee8026b250069547f3d441e1eac2f15b50c825f8f2112bfad65bf4739",
        "file_count": 38,
        "total_bytes": 3614233,
    },
    "preliminary": {
        "manifest_sha256":
            "55bf0586f414a469ddb0b434784adb2008239664d0a197deee61fbbae98f7ce4",
        "tree_sha256":
            "777edd42225a2feef23edfabab558aff980dc08e92dbe88bbbe85772eb685dc8",
        "file_count": 39,
        "total_bytes": 3597231,
    },
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path):
    path = Path(path)
    return {"bytes": path.stat().st_size, "sha256": sha256(path)}


def tree_hash(root):
    root = Path(root)
    digest = hashlib.sha256()
    files = sorted(path for path in root.rglob("*") if path.is_file())
    for path in files:
        digest.update(path.relative_to(root).as_posix().encode("utf-8") + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
        digest.update(b"\n")
    return {
        "sha256": digest.hexdigest(),
        "file_count": len(files),
        "total_bytes": sum(path.stat().st_size for path in files),
    }


def canonical_protected_tree(snapshot):
    payload = {
        entry["path"]: {
            "type": entry["type"],
            "sha256": entry["sha256"],
            "size_bytes": entry["size_bytes"],
        }
        for entry in snapshot["protected_files"]
    }
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def git(*args):
    completed = subprocess.run(
        ["git"] + list(args), cwd=str(REPO_ROOT), check=True,
        capture_output=True, text=True)
    return completed.stdout


def main():
    if not (RUN / "CLAUDE_REPORT.md").is_file():
        raise SystemExit("CLAUDE_REPORT.md is missing")
    if not (RUN / "COMMANDS.md").is_file():
        raise SystemExit("COMMANDS.md is missing")
    if not (RUN / "logs" / "source_manifest_generation.log").is_file():
        raise SystemExit("generation log is missing; run through tee")

    old_submitted_manifest = json.loads(
        (OLD_SUBMITTED / "source_manifest.json").read_text())
    source_paths = sorted(old_submitted_manifest["source_files"])
    if REWORK_DOC not in source_paths:
        source_paths.append(REWORK_DOC)
    source_paths = sorted(source_paths)
    if len(source_paths) != 15:
        raise SystemExit("expected 15 source files, got {}".format(
            len(source_paths)))
    source_files = {
        relative: file_record(REPO_ROOT / relative)
        for relative in source_paths}

    run_files = {}
    for path in sorted(RUN.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(RUN).as_posix()
        if relative in EXCLUSIONS:
            continue
        run_files[relative] = file_record(path)
    run_total_bytes = sum(record["bytes"] for record in run_files.values())

    submitted_tree = tree_hash(OLD_SUBMITTED)
    preliminary_tree = tree_hash(OLD_PRELIMINARY)
    for key, root, tree in (
            ("submitted", OLD_SUBMITTED, submitted_tree),
            ("preliminary", OLD_PRELIMINARY, preliminary_tree)):
        expected = EXPECTED_OLD[key]
        actual_manifest = sha256(root / "source_manifest.json")
        if actual_manifest != expected["manifest_sha256"]:
            raise SystemExit("{} old source_manifest changed".format(key))
        for field in ("sha256", "file_count", "total_bytes"):
            target = expected["tree_sha256"] if field == "sha256" else (
                expected["file_count"] if field == "file_count"
                else expected["total_bytes"])
            if tree[field] != target:
                raise SystemExit("{} old tree {} changed: {} != {}".format(
                    key, field, tree[field], target))

    g0_snapshot = json.loads(G0_PATH.read_text())
    before_path = RUN / "legacy" / "protected_before.json"
    after_path = RUN / "legacy" / "protected_after.json"
    before = json.loads(before_path.read_text())
    after = json.loads(after_path.read_text())
    if before["protected_tree_sha256"] != canonical_protected_tree(before):
        raise SystemExit("before protected tree hash is inconsistent")
    if after["protected_tree_sha256"] != canonical_protected_tree(after):
        raise SystemExit("after protected tree hash is inconsistent")
    if before["protected_tree_sha256"] != after["protected_tree_sha256"]:
        raise SystemExit("protected tree changed within this run")
    if before["protected_file_count"] != after["protected_file_count"]:
        raise SystemExit("protected file count changed within this run")

    protection_path = RUN / "protection_diff_vs_g0.json"
    protection = json.loads(protection_path.read_text())
    if not protection["authorization"]["only_authorized_changes"]:
        raise SystemExit("protection whitelist mismatch")
    added = sorted(row["path"] for row in protection["added_vs_g0"])
    changed = sorted(row["path"] for row in protection["changed_vs_g0"])
    if protection["removed_vs_g0"]:
        raise SystemExit("protected files were removed")
    if added != sorted(protection["authorization"]["allowed_new_files"]):
        raise SystemExit("addition list is not exactly the G1 whitelist")
    if changed != protection["authorization"]["allowed_tracked_changes"]:
        raise SystemExit("tracked change list is not exactly setup.py")

    results = json.loads((RUN / "results.json").read_text())
    evidence = json.loads((RUN / "analysis" / "budget_evidence.json").read_text())
    benchmark = json.loads((RUN / "benchmark" / "summary.json").read_text())
    analytic = evidence["analytic_double_lane_change"]
    c2 = evidence["c2_nonzero_g"]
    if results["status"] != "PASS_PENDING_CODEX_RECHECK":
        raise SystemExit("results status is not the expected pending status")
    if results["codex_acceptance"] != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("codex acceptance was self-claimed")
    if results["scope"]["g2_entered"]:
        raise SystemExit("G2 was entered")
    if len(benchmark["checks"]) != 26 or any(
            not check.get("pass") for check in benchmark["checks"]):
        raise SystemExit("benchmark checks are not 26/26 pass")
    if results["tests"]["profile"]["count"] != 22:
        raise SystemExit("profile test count is not 22")
    if results["tests"]["geometry"]["count"] != 20:
        raise SystemExit("geometry test count is not 20")
    if analytic["budget"]["position_error_bound_m"] != 7.737011020460756e-08:
        raise SystemExit("analytic position budget changed")
    if c2["budget"]["position_error_bound_m"] != 9.779693223310183e-09:
        raise SystemExit("C2 position budget changed")
    if results["previous_evidence"]["submitted_run"]["tree"]["sha256"] != \
            submitted_tree["sha256"]:
        raise SystemExit("results old submitted tree does not match")
    if results["previous_evidence"]["preliminary_run"]["tree"]["sha256"] != \
            preliminary_tree["sha256"]:
        raise SystemExit("results old preliminary tree does not match")

    new_hash_keys = {
        "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
        "formation_continuous_a/bounds.py": "bounds.py",
        "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
        "formation_continuous_a/geometry.py": "geometry.py",
        "tests/test_virtual_a_profile.py":
            "tests/test_virtual_a_profile.py",
        "tools/virtual_a_benchmark.py": "tools/virtual_a_benchmark.py",
    }
    for relative, key in new_hash_keys.items():
        if source_files[relative]["sha256"] != results["previous_evidence"][
                "new_hashes"][key]:
            raise SystemExit("results new hash differs for " + relative)

    status = git("status", "--short")
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    staged_stat = git("diff", "--cached", "--stat")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD"],
        cwd=str(REPO_ROOT), check=False).returncode
    if head != BASELINE_HEAD or branch != "feature/virtual-rigid-formation-a":
        raise SystemExit("unexpected git head or branch")
    if ancestor != 0:
        raise SystemExit("baseline is not an ancestor")
    if staged:
        raise SystemExit("staged changes are present")

    old_source_hashes = {
        path: old_submitted_manifest["source_files"][path]["sha256"]
        for path in sorted(old_submitted_manifest["source_files"])}
    changed_sources = {}
    for path in ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
                 "formation_continuous_a/bounds.py",
                 "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
                 "formation_continuous_a/geometry.py",
                 "tests/test_virtual_a_profile.py",
                 "tools/virtual_a_benchmark.py"):
        changed_sources[path] = {
            "old_sha256": old_source_hashes[path],
            "new_sha256": source_files[path]["sha256"],
            "changed": old_source_hashes[path] != source_files[path]["sha256"],
        }

    manifest = {
        "schema": SCHEMA,
        "stage": STAGE,
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN_ID,
        "repo_root": str(REPO_ROOT),
        "baseline_head": BASELINE_HEAD,
        "excluded_self_referential": EXCLUSIONS,
        "exclusion_reasons": {
            "source_manifest.json":
                "the manifest cannot hash itself",
            "logs/source_manifest_verification.log":
                "the verifier writes this log while checking the manifest",
            "logs/source_manifest_generation.log":
                "the generator writes this log while scanning the run",
        },
        "scope_note": (
            "Source and run evidence for G1_REWORK_01. HEAD remains the "
            "baseline; no commit, push or G2 entry. Old G1 evidence is "
            "preserved and bound by its own tree hashes."),
        "source_files": source_files,
        "source_file_count": len(source_files),
        "run_files": run_files,
        "run_file_count": len(run_files),
        "run_total_bytes": run_total_bytes,
        "git": {
            "head": head,
            "branch": branch,
            "baseline_head": BASELINE_HEAD,
            "baseline_is_ancestor": True,
            "committed": "NO",
            "pushed": "NO",
            "status_short": status.splitlines(),
            "staged_diff_sha256": hashlib.sha256(
                staged.encode("utf-8")).hexdigest(),
            "staged_diff_stat": staged_stat.strip(),
        },
        "protected": {
            "g0_path": str(G0_PATH.relative_to(REPO_ROOT)),
            "g0_sha256": sha256(G0_PATH),
            "g0_tree_sha256": g0_snapshot["protected_tree_sha256"],
            "g0_file_count": g0_snapshot["protected_file_count"],
            "before_path": str(before_path.relative_to(REPO_ROOT)),
            "before_sha256": sha256(before_path),
            "before_tree_sha256": before["protected_tree_sha256"],
            "before_file_count": before["protected_file_count"],
            "after_path": str(after_path.relative_to(REPO_ROOT)),
            "after_sha256": sha256(after_path),
            "after_tree_sha256": after["protected_tree_sha256"],
            "after_file_count": after["protected_file_count"],
            "unchanged_within_run": True,
            "snapshot_files_byte_identical": (
                sha256(before_path) == sha256(after_path)),
            "added_vs_g0": added,
            "removed_vs_g0": protection["removed_vs_g0"],
            "changed_vs_g0": changed,
            "only_authorized_changes":
                protection["authorization"]["only_authorized_changes"],
            "comparison_path": "protection_diff_vs_g0.json",
            "comparison_sha256": sha256(protection_path),
            "setup_diff_stat": protection["setup_diff"]["stat"],
        },
        "results": {
            "status": results["status"],
            "codex_acceptance": results["codex_acceptance"],
            "geometry_tests": results["tests"]["geometry"]["count"],
            "profile_tests": results["tests"]["profile"]["count"],
            "benchmark_checks": len(benchmark["checks"]),
            "benchmark_failed": sum(
                not check.get("pass") for check in benchmark["checks"]),
            "analytic_position_budget_m":
                analytic["budget"]["position_error_bound_m"],
            "analytic_angle_budget_rad":
                analytic["budget"]["angle_error_bound_rad"],
            "c2_position_budget_m":
                c2["budget"]["position_error_bound_m"],
            "c2_worst_quadrature_error_m":
                c2["worst_position_error_m"],
            "analytic_worst_201_station_error_m":
                results["integration_budget"]["analytic_double_lane_change"][
                    "benchmark_201_station_worst_quadrature_error_m"],
            "baseline_unmodified_gate":
                results["tests"]["baseline_geometry_regression"][
                    "unmodified_runner_attempt"]["status"],
            "baseline_wrapper":
                results["tests"]["baseline_geometry_regression"][
                    "documented_wrapper_run"]["status"],
            "g2_entered": False,
            "nonclaims": results["constraints"],
        },
        "previous_evidence": {
            "submitted_run": {
                "path": str(OLD_SUBMITTED.relative_to(REPO_ROOT)),
                "source_manifest_sha256": EXPECTED_OLD["submitted"][
                    "manifest_sha256"],
                "tree": submitted_tree,
                "status": "PRESERVED_NOT_MODIFIED",
            },
            "preliminary_run": {
                "path": str(OLD_PRELIMINARY.relative_to(REPO_ROOT)),
                "source_manifest_sha256": EXPECTED_OLD["preliminary"][
                    "manifest_sha256"],
                "tree": preliminary_tree,
                "status": "PRESERVED_NOT_MODIFIED",
            },
            "old_declared_source_hashes": old_source_hashes,
            "changed_sources_vs_old": changed_sources,
        },
        "verification": {
            "script": "analysis/verify_source_manifest.py",
            "command": (
                "PYTHONDONTWRITEBYTECODE=1 python3 -B -I "
                "$RUN/analysis/verify_source_manifest.py"),
            "log": "logs/source_manifest_verification.log",
            "rule": (
                "Recompute every run and source SHA-256, rehash every "
                "protected snapshot entry from disk, recompute the canonical "
                "protected tree, recheck the G0 whitelist, results/nonclaims, "
                "setup registration, git state and both old evidence trees."),
        },
    }
    target = RUN / "source_manifest.json"
    target.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "status": "WRITTEN",
        "run_id": RUN_ID,
        "manifest": str(target.relative_to(REPO_ROOT)),
        "run_file_count": len(run_files),
        "run_total_bytes": run_total_bytes,
        "source_file_count": len(source_files),
        "excluded_self_referential": EXCLUSIONS,
        "submitted_old_tree": submitted_tree["sha256"],
        "preliminary_old_tree": preliminary_tree["sha256"],
        "manifest_sha256": sha256(target),
        "manifest_bytes": target.stat().st_size,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()