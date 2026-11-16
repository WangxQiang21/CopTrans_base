"""Write the G4 source-and-evidence manifest.

Binds the Codex-accepted G3 source manifest and protected tree, the 31 G3
source files plus the 39 G4 source/fixture additions, every evidence-run file
except its own three self-referential artifacts, the accepted G1/G2/G3
evidence, the G4 protection comparison, the frozen six-scene benchmark and the
open negative-case coverage. The stdlib verifier recomputes all of it.
"""
import datetime
import hashlib
import json
import re
from pathlib import Path
import subprocess

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
RUN_ID = RUN.name
SCHEMA = "virtual_rigid_a.g4.source_manifest.v1"
STAGE = "G4"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
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
EXPECTED_PROTECTED_TREE = (
    "ccae8abae3ea69470bdb7a4edfc4b95b448350f64a765facbc7f269b44ebf7c0")
EXPECTED_SCENE_MANIFEST_FILE_SHA = (
    "f956e8316ebf1e72102d89338b88bbcabe48bae5aa52977deef3670abe24240d")
EXPECTED_SCENE_MANIFEST_SELF_SHA = (
    "5bb3df2d829ef3b1051880fb916462cca60cd8c5a17e0c9174c8bb18370daa80")
EXPECTED_BENCHMARK_SUMMARY_SHA = (
    "8c3b7b1d9c5719fd59e8d5205983da83567b2b81eeb614a29e23da4c841ccb1d")
EXPECTED_RESULTS_SHA = (
    "9c3995d6d24243533912cb5459721e6bebe5ca494bb091d72acd8ff3bd278d0e")
EXPECTED_NUMERICS_SHA = (
    "0c36947d0f6c6df5cd366615614cd571d82597dfa35f2119cf623b22846c0790")
EXPECTED_ENVIRONMENT_SHA = (
    "4bd6c680cead8a8b66d5cb18af8dc0903240b740ae7718e71c5b2b0053d349dd")
EXPECTED_PROTECTION_SHA = (
    "3781f956e290d0bd800c8c7c5856071443666ab4bde1a3555a0781719711cd83")
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
SCENE_NAMES = ("S1", "S1_mirror", "S2", "S2_mirror", "S3", "S3_mirror")
SYNTHETIC_NAMES = (
    "arc_corner", "boundary_goal", "blocked_start", "corridor",
    "door_circumcircle", "thin_barrier", "unknown_gray")
G4_NEW_CODE = sorted([
    PACKAGE + "collision.py",
    PACKAGE + "integration_bounds.py",
    PACKAGE + "pipeline.py",
    "docs/virtual_formation_a/CLAUDE_G4_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G4_TASK.md",
    "tests/test_virtual_a_collision.py",
    "tests/test_virtual_a_integration_bounds.py",
    "tests/test_virtual_a_pipeline.py",
    "tests/virtual_a_g4_support.py",
    "tools/virtual_a_scenes.py",
])
G4_NEW_FIXTURES = sorted(
    ["tests/fixtures/virtual_a/maps/scenes/{}.{}".format(
        name, extension)
     for name in SCENE_NAMES for extension in ("pgm", "yaml")] +
    ["tests/fixtures/virtual_a/maps/synthetic/{}/map.{}".format(
        name, extension)
     for name in SYNTHETIC_NAMES for extension in ("pgm", "yaml")] +
    ["tests/fixtures/virtual_a/scenes/{}.request.json".format(name)
     for name in SCENE_NAMES])
EXPECTED_CHANGED = sorted([
    PACKAGE + "audit.py",
    PACKAGE + "contracts.py",
    PACKAGE + "optimizer.py",
    PACKAGE + "trajectory.py",
    "tools/virtual_a_benchmark.py",
])
EXPECTED_ADDED_PROTECTED = sorted(
    [PACKAGE + "collision.py", PACKAGE + "integration_bounds.py",
     PACKAGE + "pipeline.py"] +
    [path for path in G4_NEW_FIXTURES] +
    ["tests/test_virtual_a_collision.py",
     "tests/test_virtual_a_integration_bounds.py",
     "tests/test_virtual_a_pipeline.py",
     "tests/virtual_a_g4_support.py",
     "tools/virtual_a_scenes.py"])
EXPECTED_SOURCE_COUNT = 73
EXPECTED_TEST_TOTAL = 191
EXPECTED_SCENES = list(SCENE_NAMES)
EXPECTED_REGRESSION_CHECKS = {"G1": 26, "G2": 124, "G3": 151}
EXCLUSIONS = [
    "source_manifest.json",
    "logs/source_manifest_generation.log",
    "logs/source_manifest_verification.log",
]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path):
    path = Path(path)
    return {"bytes": path.stat().st_size, "sha256": sha256(path)}


def canonical_tree(entries):
    payload = {
        entry["path"]: {
            "type": entry["type"],
            "sha256": entry["sha256"],
            "size_bytes": entry["size_bytes"],
        }
        for entry in entries
    }
    return hashlib.sha256(json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")).hexdigest()


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(REPO_ROOT), check=True,
        capture_output=True, text=True).stdout


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def rows_from_snapshot(snapshot):
    return {row["path"]: dict(row) for row in snapshot["protected_files"]}


def apply_diff(rows, diff, sha_key, size_key):
    for row in diff:
        rows[row["path"]] = {
            "path": row["path"], "type": "file",
            "sha256": row[sha_key], "size_bytes": row[size_key]}
    return rows


def verify_g1_g2(g1_snapshot, g2_manifest, g2_protection):
    if sha256(G1_PROTECTED) != EXPECTED_G1_AFTER_SHA:
        raise SystemExit("accepted G1 protected_after.json changed")
    if g1_snapshot["protected_tree_sha256"] != EXPECTED_G1_TREE:
        raise SystemExit("G1 protected tree changed")
    if canonical_tree(g1_snapshot["protected_files"]) != EXPECTED_G1_TREE:
        raise SystemExit("G1 snapshot is internally inconsistent")
    acceptance = load(G2_ACCEPTANCE)
    if acceptance.get("status") != "ACCEPTED":
        raise SystemExit("G2 acceptance is not ACCEPTED")
    if acceptance.get("source_manifest_sha256") != EXPECTED_G2_MANIFEST_SHA:
        raise SystemExit("G2 acceptance does not bind its manifest")
    if acceptance.get("next_stage_authorized") is not False:
        raise SystemExit("G2 acceptance unexpectedly authorizes G3")
    if sha256(G2_MANIFEST) != EXPECTED_G2_MANIFEST_SHA:
        raise SystemExit("accepted G2 source manifest changed")
    if g2_manifest["protected"][
            "current_full_protected_tree_sha256"] != EXPECTED_G2_TREE:
        raise SystemExit("G2 manifest protected tree changed")
    if g2_protection["current"][
            "full_protected_tree_sha256"] != EXPECTED_G2_TREE:
        raise SystemExit("G2 protection diff tree changed")
    if g2_protection["authorization"][
            "only_authorized_changes"] is not True:
        raise SystemExit("G2 protection diff was not authorized")
    for relative, record in acceptance["source_sha256"].items():
        source = g2_manifest["source_files"].get(relative)
        if source is None or source["sha256"] != record:
            raise SystemExit("G2 acceptance source hash is unbound")
    g2_rows = rows_from_snapshot(g1_snapshot)
    apply_diff(g2_rows, g2_protection["added_vs_g1_accepted"],
               "sha256", "size_bytes")
    apply_diff(g2_rows, g2_protection["changed_vs_g1_accepted"],
               "g2_sha256", "g2_size_bytes")
    if canonical_tree(g2_rows.values()) != EXPECTED_G2_TREE:
        raise SystemExit("G2 protected-tree reconstruction failed")
    return acceptance, g2_rows


def verify_g3(g3_manifest, g3_protection, g2_rows):
    acceptance = load(G3_ACCEPTANCE)
    if acceptance.get("status") != "ACCEPTED":
        raise SystemExit("G3 acceptance is not ACCEPTED")
    if acceptance.get("source_manifest_sha256") != EXPECTED_G3_MANIFEST_SHA:
        raise SystemExit("G3 acceptance does not bind its manifest")
    if acceptance.get("next_stage_authorized") is not False:
        raise SystemExit("G3 acceptance authorizes a later stage")
    if sha256(G3_MANIFEST) != EXPECTED_G3_MANIFEST_SHA:
        raise SystemExit("accepted G3 source manifest changed")
    if g3_manifest["protected"][
            "current_full_protected_tree_sha256"] != EXPECTED_G3_TREE:
        raise SystemExit("G3 manifest protected tree changed")
    if g3_protection["current"][
            "full_protected_tree_sha256"] != EXPECTED_G3_TREE:
        raise SystemExit("G3 protection diff tree changed")
    if g3_protection["authorization"][
            "only_authorized_changes"] is not True:
        raise SystemExit("G3 protection diff was not authorized")
    if g3_protection["removed_vs_g2_accepted"]:
        raise SystemExit("G3 records protected removals")
    for relative, record in acceptance["source_sha256"].items():
        source = g3_manifest["source_files"].get(relative)
        if source is None or source["sha256"] != record:
            raise SystemExit("G3 acceptance source hash is unbound")
    excluded = set(g3_manifest.get("excluded_self_referential", []))
    checked = {}
    for relative, record in sorted(g3_manifest["run_files"].items()):
        if relative in excluded:
            continue
        path = G3_RUN / relative
        if not path.is_file() or sha256(path) != record["sha256"]:
            raise SystemExit("accepted G3 run file changed: " + relative)
        checked[relative] = record
    g3_rows = {path: dict(row) for path, row in g2_rows.items()}
    apply_diff(g3_rows, g3_protection["added_vs_g2_accepted"],
               "sha256", "size_bytes")
    apply_diff(g3_rows, g3_protection["changed_vs_g2_accepted"],
               "g3_sha256", "g3_size_bytes")
    if canonical_tree(g3_rows.values()) != EXPECTED_G3_TREE:
        raise SystemExit("G3 protected-tree reconstruction failed")
    return acceptance, g3_rows, checked, excluded


def parse_test(name, expected):
    path = RUN / "logs" / (name + ".log")
    text = path.read_text(encoding="utf-8")
    match = re.search(r"Ran (\d+) tests? in ([0-9.]+)s", text)
    if match is None or int(match.group(1)) != expected or \
            text.rstrip().splitlines()[-1].strip() != "OK":
        raise SystemExit("{} test log is not the expected OK run".format(name))
    return {"count": int(match.group(1)),
            "wall_time_s": float(match.group(2)),
            "log_sha256": sha256(path)}


def main():
    for required in (
            "CLAUDE_REPORT.md", "COMMANDS.md", "environment.json",
            "results.json", "scenes/scene_manifest.json",
            "benchmark/summary.json", "benchmark/results.json",
            "analysis/protection_diff_vs_g3_accepted.py",
            "analysis/protection_diff_vs_g3_accepted.json",
            "analysis/g4_numerics.py", "analysis/g4_numerics.json",
            "analysis/write_environment.py", "analysis/assemble_evidence.py",
            "analysis/verify_source_manifest.py",
            "logs/exit_codes.log", "logs/regression_exit_codes.log",
            "logs/scenes.log", "logs/collision.log", "logs/pipeline.log",
            "logs/integration_bounds.log",
            "logs/benchmark.log", "logs/g1_g2_g3_regression.log",
            "logs/regression_g1_benchmark.log",
            "logs/regression_g2_benchmark.log",
            "logs/regression_g3_benchmark.log",
            "logs/protection_diff.log", "logs/g4_numerics.log",
            "logs/environment.log", "logs/assemble_evidence.log"):
        if not (RUN / required).is_file():
            raise SystemExit("missing evidence before manifest: " + required)
    if [path for path in RUN.rglob("*")
            if path.name == "__pycache__" or
            path.suffix in (".pyc", ".pyo")]:
        raise SystemExit("generated bytecode exists under the run")

    g1_snapshot = load(G1_PROTECTED)
    g2_manifest = load(G2_MANIFEST)
    g2_protection = load(G2_PROTECTION)
    _g2_acceptance, g2_rows = verify_g1_g2(
        g1_snapshot, g2_manifest, g2_protection)
    g3_manifest = load(G3_MANIFEST)
    g3_protection = load(G3_PROTECTION)
    g3_acceptance, g3_rows, g3_checked, g3_excluded = verify_g3(
        g3_manifest, g3_protection, g2_rows)

    g3_sources = dict(g3_manifest["source_files"])
    source_paths = sorted(set(g3_sources) | set(G4_NEW_CODE) |
                          set(G4_NEW_FIXTURES))
    if len(source_paths) != EXPECTED_SOURCE_COUNT:
        raise SystemExit("expected {} source files, got {}".format(
            EXPECTED_SOURCE_COUNT, len(source_paths)))
    missing = [path for path in source_paths
               if not (REPO_ROOT / path).is_file()]
    if missing:
        raise SystemExit("source files missing: {}".format(missing[:3]))
    source_files = {
        relative: file_record(REPO_ROOT / relative)
        for relative in source_paths}
    changed_sources = {}
    for relative in sorted(g3_sources):
        old_hash = g3_sources[relative]["sha256"]
        new_hash = source_files[relative]["sha256"]
        changed_sources[relative] = {
            "g3_sha256": old_hash,
            "g4_sha256": new_hash,
            "changed": old_hash != new_hash,
        }
    actually_changed = sorted(
        path for path, record in changed_sources.items() if record["changed"])
    if actually_changed != EXPECTED_CHANGED:
        raise SystemExit("unexpected changes to accepted G3 sources: "
                         "{}".format(actually_changed))
    for relative in G4_NEW_CODE + G4_NEW_FIXTURES:
        if relative in g3_sources:
            raise SystemExit("new G4 source was already accepted: " + relative)

    protection_path = (RUN / "analysis" /
                       "protection_diff_vs_g3_accepted.json")
    protection = load(protection_path)
    if sha256(protection_path) != EXPECTED_PROTECTION_SHA:
        raise SystemExit("G4 protection diff changed")
    if not protection["authorization"]["only_authorized_changes"]:
        raise SystemExit("G4 protection diff is not exactly authorized")
    if protection["removed_vs_g3_accepted"]:
        raise SystemExit("G4 records protected removals")
    added = sorted(row["path"] for row in
                   protection["added_vs_g3_accepted"])
    changed = sorted(row["path"] for row in
                     protection["changed_vs_g3_accepted"])
    if added != EXPECTED_ADDED_PROTECTED:
        raise SystemExit("G4 protected additions differ from the whitelist")
    if changed != EXPECTED_CHANGED:
        raise SystemExit("G4 protected changes differ from the whitelist")
    if protection["current"][
            "full_protected_tree_sha256"] != EXPECTED_PROTECTED_TREE:
        raise SystemExit("current protected tree changed")
    if protection["current"]["unchanged_common_tree_matches"] is not True:
        raise SystemExit("common protected tree changed")
    if protection["g3_evidence"]["protected_tree_sha256"] != EXPECTED_G3_TREE:
        raise SystemExit("protection diff G3 tree changed")

    scene_manifest_path = RUN / "scenes/scene_manifest.json"
    if sha256(scene_manifest_path) != EXPECTED_SCENE_MANIFEST_FILE_SHA:
        raise SystemExit("scene manifest file changed")
    scene_manifest = load(scene_manifest_path)
    if scene_manifest["manifest_sha256"] != EXPECTED_SCENE_MANIFEST_SELF_SHA:
        raise SystemExit("scene manifest self digest changed")
    if scene_manifest["frozen"] is not True or \
            scene_manifest["reference_verified"] is not True or \
            scene_manifest["failures"] != []:
        raise SystemExit("scene manifest is not a clean freeze")
    if sorted(scene_manifest["scenes"]) != sorted(EXPECTED_SCENES):
        raise SystemExit("scene set changed")
    for name in EXPECTED_SCENES:
        entry = scene_manifest["scenes"][name]
        if entry["status"] != "REFERENCE_PASSED":
            raise SystemExit("scene reference not passed: " + name)
        input_dir = RUN / "scenes" / entry["input_dir"]
        files = sorted(path.name for path in input_dir.iterdir()
                       if path.is_file())
        if files != ["map.pgm", "map.yaml", "request.json"]:
            raise SystemExit("input bundle changed: " + name)

    benchmark_path = RUN / "benchmark/summary.json"
    if sha256(benchmark_path) != EXPECTED_BENCHMARK_SUMMARY_SHA:
        raise SystemExit("benchmark summary changed")
    benchmark = load(benchmark_path)
    if benchmark["status"] != "PASS" or len(benchmark["checks"]) != 84 or \
            any(not check["pass"] for check in benchmark["checks"]):
        raise SystemExit("benchmark is not 84/84 PASS")
    if benchmark["codex_acceptance"] != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("benchmark self-claims Codex acceptance")
    if benchmark["manifest_sha256"] != scene_manifest["manifest_sha256"]:
        raise SystemExit("benchmark manifest binding changed")
    benchmark_results = load(RUN / "benchmark/results.json")
    if sorted(benchmark_results) != sorted(EXPECTED_SCENES):
        raise SystemExit("benchmark result scenes changed")
    for name in EXPECTED_SCENES:
        record = benchmark_results[name]
        if record["status"] != "SUCCESS" or record["executable"] is not True:
            raise SystemExit("scene is not executable: " + name)
        if record["manifest_map_match"] is not True or \
                record["manifest_request_match"] is not True or \
                record["reference_free_bundle"] is not True:
            raise SystemExit("scene input binding changed: " + name)
        if [record["request_sha256"], record["grid_sha256"],
                record["planning_context_sha256"]] != [
                scene_manifest["scenes"][name]["request_sha256"],
                scene_manifest["scenes"][name]["map_sha256"],
                scene_manifest["scenes"][name]["planning_context_sha256"]]:
            raise SystemExit("scene digest binding changed: " + name)
        for layer in ("geometry", "dynamics", "collision"):
            if record["audits"][layer]["passed"] is not True:
                raise SystemExit("{} audit failed: {}".format(layer, name))
        if hashlib.sha256(
                record["model_json"].encode("utf-8")).hexdigest() != \
                record["model_sha256"]:
            raise SystemExit("scene model digest changed: " + name)

    results_path = RUN / "results.json"
    if sha256(results_path) != EXPECTED_RESULTS_SHA:
        raise SystemExit("results.json changed")
    results = load(results_path)
    if results["status"] != "PASS_PENDING_CODEX_RECHECK":
        raise SystemExit("results status is not pending Codex recheck")
    if results["codex_acceptance"] != "NOT_RUN_NOT_CLAIMED":
        raise SystemExit("results self-claim Codex acceptance")
    if len(results["scenes"]) != 6 or \
            not all(record["executable"] for record in
                    results["scenes"].values()):
        raise SystemExit("results scene records changed")
    if results["autonomous_benchmark"]["checks"] != 84 or \
            results["autonomous_benchmark"]["status"] != "PASS":
        raise SystemExit("results benchmark record changed")
    if results["attempt_summary"]["total_attempts"] != 48 or \
            results["attempt_summary"]["all_attempts_retained"] is not True:
        raise SystemExit("attempt summary changed")
    if [case["case"] for case in
            results["numbered_negative_coverage"]] != list(range(1, 16)):
        raise SystemExit("numbered negative coverage changed")
    if any(not case["observed_passing"] for case in
           results["numbered_negative_coverage"]):
        raise SystemExit("a numbered negative was not observed passing")
    regressions = {record["stage"]: record["checks"]
                   for record in results["regression_benchmarks"]}
    if regressions != EXPECTED_REGRESSION_CHECKS:
        raise SystemExit("regression benchmark counts changed")
    if results["tests"]["total_tests"] != EXPECTED_TEST_TOTAL:
        raise SystemExit("test total changed")
    for rel, expected_sha in (
            ("analysis/g4_numerics.json", EXPECTED_NUMERICS_SHA),
            ("environment.json", EXPECTED_ENVIRONMENT_SHA)):
        if sha256(RUN / rel) != expected_sha:
            raise SystemExit("evidence file changed: " + rel)
    if results["protection"]["full_protected_tree_sha256"] != \
            EXPECTED_PROTECTED_TREE:
        raise SystemExit("results protected tree changed")

    test_counts = {}
    for name, expected in (("collision", 17), ("pipeline", 8),
                           ("integration_bounds", 6)):
        test_counts[name] = parse_test(name, expected)
    regression_text = (
        RUN / "logs/g1_g2_g3_regression.log").read_text(encoding="utf-8")
    for expected in (20, 26, 22, 51, 18, 23):
        if "Ran {} tests".format(expected) not in regression_text:
            raise SystemExit("regression count missing: {}".format(expected))
    if regression_text.count("\nOK\n") != 6:
        raise SystemExit("regression suites did not all finish OK")

    run_files = {}
    for path in sorted(RUN.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(RUN).as_posix()
        if relative in EXCLUSIONS:
            continue
        run_files[relative] = file_record(path)
    run_total_bytes = sum(record["bytes"] for record in run_files.values())

    status = git("status", "--short")
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD"],
        cwd=str(REPO_ROOT), check=False).returncode
    if head != BASELINE_HEAD or branch != "feature/virtual-rigid-formation-a":
        raise SystemExit("unexpected git HEAD or branch")
    if ancestor != 0:
        raise SystemExit("baseline is not an ancestor")
    if staged:
        raise SystemExit("staged changes are present")

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
            "source_manifest.json": "the manifest cannot hash itself",
            "logs/source_manifest_generation.log":
                "the generator writes this log while scanning the run",
            "logs/source_manifest_verification.log":
                "the verifier writes this log while checking the manifest",
        },
        "scope_note": (
            "G4 only: frozen A-only real-map scenes, conservative swept "
            "collision adaptation, route guidance and autonomous pipeline. "
            "SUCCESS/executable means the three audits passed for these six "
            "known-feasible finite-map requests; it is not a general success "
            "rate, completeness, optimality, real-time or Codex acceptance "
            "claim. This run is G4 rework 01: the collision envelope uses the "
            "actual legacy per-side half-extent semantics (factor 2), and "
            "the independent audit threshold is restored to 5e-8 m with an "
            "exact per-panel fourth-derivative interval bound. HEAD is "
            "unchanged and no commit, push, branch or G5 entry occurred."),
        "source_files": source_files,
        "source_file_count": len(source_files),
        "g4_new_source_files": G4_NEW_CODE + G4_NEW_FIXTURES,
        "g4_new_code_and_test_files": G4_NEW_CODE,
        "g4_new_fixture_files": G4_NEW_FIXTURES,
        "changed_accepted_sources": changed_sources,
        "changed_accepted_source_list": actually_changed,
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
            "branch_created": "NO",
            "status_short": status.splitlines(),
            "staged_diff_sha256": hashlib.sha256(
                staged.encode("utf-8")).hexdigest(),
        },
        "protected": {
            "g1_protected_after_path":
                str(G1_PROTECTED.relative_to(REPO_ROOT)),
            "g1_protected_after_sha256": sha256(G1_PROTECTED),
            "g1_tree_sha256": g1_snapshot["protected_tree_sha256"],
            "g1_file_count": g1_snapshot["protected_file_count"],
            "g2_acceptance_path": str(G2_ACCEPTANCE.relative_to(REPO_ROOT)),
            "g2_acceptance_sha256": sha256(G2_ACCEPTANCE),
            "g2_run": str(G2_RUN.relative_to(REPO_ROOT)),
            "g2_manifest_sha256": sha256(G2_MANIFEST),
            "g2_tree_sha256": EXPECTED_G2_TREE,
            "g3_acceptance_path": str(G3_ACCEPTANCE.relative_to(REPO_ROOT)),
            "g3_acceptance_sha256": sha256(G3_ACCEPTANCE),
            "g3_acceptance_status": g3_acceptance["status"],
            "g3_acceptance_next_stage_authorized":
                g3_acceptance["next_stage_authorized"],
            "g3_run": str(G3_RUN.relative_to(REPO_ROOT)),
            "g3_manifest_sha256": sha256(G3_MANIFEST),
            "g3_manifest_run_files_checked": len(g3_checked),
            "g3_manifest_run_files_all_match": True,
            "g3_excluded_self_referential": sorted(g3_excluded),
            "g3_tree_sha256": EXPECTED_G3_TREE,
            "g3_file_count": len(g3_rows),
            "g3_acceptance_source_hashes_bound": len(
                g3_acceptance["source_sha256"]),
            "comparison_path": str(protection_path.relative_to(RUN)),
            "comparison_sha256": sha256(protection_path),
            "added_vs_g3_accepted": added,
            "changed_vs_g3_accepted": changed,
            "removed_vs_g3_accepted": protection["removed_vs_g3_accepted"],
            "only_authorized_changes": True,
            "current_full_protected_tree_sha256":
                protection["current"]["full_protected_tree_sha256"],
            "unchanged_common_tree_sha256":
                protection["current"][
                    "unchanged_common_tree_sha256_current"],
            "current_file_count": protection["current"]["current_file_count"],
        },
        "scene_freeze": {
            "manifest_path": "scenes/scene_manifest.json",
            "manifest_sha256": sha256(scene_manifest_path),
            "manifest_self_digest": scene_manifest["manifest_sha256"],
            "scene_count": len(scene_manifest["scenes"]),
            "scenes": sorted(scene_manifest["scenes"]),
            "frozen": scene_manifest["frozen"],
            "reference_verified": scene_manifest["reference_verified"],
            "inputs_have_no_reference":
                scene_manifest["inputs_have_no_reference"],
            "reference_verification": scene_manifest[
                "reference_verification"],
            "per_scene_digests": {
                name: {
                    "input_dir": scene_manifest["scenes"][name]["input_dir"],
                    "map_sha256":
                        scene_manifest["scenes"][name]["map_sha256"],
                    "request_sha256":
                        scene_manifest["scenes"][name]["request_sha256"],
                    "planning_context_sha256":
                        scene_manifest["scenes"][name][
                            "planning_context_sha256"],
                    "timed_model_sha256":
                        scene_manifest["scenes"][name][
                            "timed_model_sha256"],
                    "reference_file":
                        scene_manifest["scenes"][name]["reference_file"],
                } for name in EXPECTED_SCENES},
        },
        "benchmark": {
            "summary_path": "benchmark/summary.json",
            "summary_sha256": sha256(benchmark_path),
            "results_path": "benchmark/results.json",
            "results_sha256": sha256(RUN / "benchmark/results.json"),
            "status": benchmark["status"],
            "checks": len(benchmark["checks"]),
            "failed": sum(not check["pass"]
                          for check in benchmark["checks"]),
            "executable": sum(1 for name in EXPECTED_SCENES
                              if benchmark_results[name]["executable"]),
            "scene_count": len(EXPECTED_SCENES),
            "codex_acceptance": benchmark["codex_acceptance"],
            "per_scene": {
                name: {
                    "status": benchmark_results[name]["status"],
                    "executable": benchmark_results[name]["executable"],
                    "checks": benchmark_results[name]["checks"],
                    "attempts": len(benchmark_results[name]["attempts"]),
                    "routes": len(benchmark_results[name]["routes"]),
                    "model_sha256":
                        benchmark_results[name]["model_sha256"],
                    "collision_margin_m":
                        benchmark_results[name]["collision_margin_m"],
                } for name in EXPECTED_SCENES},
        },
        "results": {
            "status": results["status"],
            "codex_acceptance": results["codex_acceptance"],
            "sha256": sha256(results_path),
            "scene_count": len(results["scenes"]),
            "attempt_summary": results["attempt_summary"],
            "numbered_negative_coverage": [
                {"case": case["case"], "tests": case["tests"],
                 "observed_passing": case["observed_passing"]}
                for case in results["numbered_negative_coverage"]],
            "tests": test_counts,
            "regression_benchmarks": results["regression_benchmarks"],
            "independent_numerics_sha256":
                results["independent_numerics"]["sha256"],
            "environment_sha256": results["environment"]["sha256"],
            "pre_final_corrections":
                results["pre_final_corrections"],
            "nonclaims": results["nonclaims"],
        },
        "historical_runs_read_only": True,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }
    target = RUN / "source_manifest.json"
    target.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "path": str(target.relative_to(REPO_ROOT)),
        "source_file_count": len(source_files),
        "g4_new_sources": len(G4_NEW_CODE) + len(G4_NEW_FIXTURES),
        "changed_accepted_sources": len(actually_changed),
        "run_file_count": len(run_files),
        "protected_tree_sha256": EXPECTED_PROTECTED_TREE,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()