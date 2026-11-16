"""Prove that only the authorised files changed and old evidence is intact.

Checks, in order:

1. The G3 and G4 Codex acceptance records are byte-identical to their recorded
   digests, still say ACCEPTED, and still bind the accepted run manifests.
2. The accepted G4 source manifest (73 files) differs from the current tree in
   exactly ``formation_continuous_a/audit.py`` and
   ``tests/test_virtual_a_audit.py``; every other accepted source is unchanged.
3. Every historical G3/G4 run that has a source manifest still matches its own
   recorded file sizes and SHA-256 digests, without writing into it.
"""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[5]
RUN = Path(__file__).resolve().parents[1]
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
AUTHORISED_CHANGED = sorted([
    PACKAGE + "audit.py",
    "tests/test_virtual_a_audit.py",
])
G3_ACCEPTANCE = (
    ROOT / "evaluation/virtual_formation_a/G3/codex_acceptance_rework_01/"
    "CODEX_ACCEPTANCE.json")
G4_ACCEPTANCE = (
    ROOT / "evaluation/virtual_formation_a/G4/codex_acceptance_rework_01/"
    "CODEX_ACCEPTANCE.json")
ACCEPTANCE_HASHES = {
    str(G3_ACCEPTANCE.relative_to(ROOT)):
        "adea82755b9fbaec7c589ff411d9b6a06b7cb23d7a89866c0edb4e5d882bff9d",
    str(G4_ACCEPTANCE.relative_to(ROOT)):
        "413f5f4cc1965bb982be8771d0ea94c41e240a0ac5c934d20812fbeef638a17a",
}
ACCEPTED_G3_MANIFEST = (
    ROOT / "evaluation/virtual_formation_a/G3/20261116T031558Z/"
    "source_manifest.json")
ACCEPTED_G4_MANIFEST = (
    ROOT / "evaluation/virtual_formation_a/G4/20261116T081458Z/"
    "source_manifest.json")
ACCEPTED_MANIFEST_HASHES = {
    str(ACCEPTED_G3_MANIFEST.relative_to(ROOT)):
        "bad6f55441bd34ca8d164cc954170336b56df398fcb0912dae10a4588279ceea",
    str(ACCEPTED_G4_MANIFEST.relative_to(ROOT)):
        "caa929d09a4a346e910a13f8c1d5b4ad4a2bfe3b9dbf2e95b0609bd81994f8c6",
}
HISTORICAL = (
    (ROOT / "evaluation/virtual_formation_a/G3/20261116T023528Z",
     "050fa2dfab1687795d8bce7621f96d9dbe46b8900131b9282a0fbd393c209adc",
     156),
    (ROOT / "evaluation/virtual_formation_a/G3/20261116T031558Z",
     "bad6f55441bd34ca8d164cc954170336b56df398fcb0912dae10a4588279ceea",
     158),
    (ROOT / "evaluation/virtual_formation_a/G4/20261116T055008Z",
     "353b1e1f8733e53a14969575ac086466edcf089ce65283d03c3f31ce886e79ef",
     78),
    (ROOT / "evaluation/virtual_formation_a/G4/20261116T081458Z",
     "caa929d09a4a346e910a13f8c1d5b4ad4a2bfe3b9dbf2e95b0609bd81994f8c6",
     81),
)
BASELINE_AUDIT_SHA = {
    "G3_accepted":
        "819a44e85a578ab91271b2ad5da1d332dcf921e483dfc946296834308415097b",
    "G4_accepted":
        "7549420ea2319aa6d153592e7b6a9b205018352969891214a97111cb3875e604",
}
BASELINE_AUDIT_TEST_SHA = {
    "G3_accepted":
        "7708c3b51404d810d6c3c055ee5979cd5401def04a892021f97c1b0267eb582f",
    "G4_manifest":
        "7708c3b51404d810d6c3c055ee5979cd5401def04a892021f97c1b0267eb582f",
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition, label):
    if not condition:
        raise SystemExit("PROTECTION CHECK FAILED: " + label)


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def verify_acceptance():
    records = {}
    for path in (G3_ACCEPTANCE, G4_ACCEPTANCE):
        relative = str(path.relative_to(ROOT))
        digest = sha256(path)
        require(digest == ACCEPTANCE_HASHES[relative],
                "acceptance record changed: " + relative)
        record = load(path)
        require(record["status"] == "ACCEPTED", "acceptance status")
        require(record["head"] == BASELINE_HEAD, "acceptance head")
        require(record["next_stage_authorized"] is False,
                "acceptance authorises a later stage")
        manifest = ROOT / record["evidence_run"] / "source_manifest.json"
        require(sha256(manifest) ==
                ACCEPTED_MANIFEST_HASHES[str(manifest.relative_to(ROOT))],
                "acceptance manifest hash")
        require(record["source_manifest_sha256"] == sha256(manifest),
                "acceptance manifest binding")
        records[record["stage"]] = {
            "path": relative,
            "sha256": digest,
            "status": record["status"],
            "head": record["head"],
            "next_stage_authorized": record["next_stage_authorized"],
            "evidence_run": record["evidence_run"],
            "source_manifest_sha256": record["source_manifest_sha256"],
            "known_nonblocking_issue": record["known_nonblocking_issue"],
            "legacy_timing_regression": record["legacy_timing_regression"],
            "audit_source_sha256": record["source_sha256"].get(
                PACKAGE + "audit.py"),
            "audit_test_source_sha256": record["source_sha256"].get(
                "tests/test_virtual_a_audit.py"),
        }
    require(records["G3"]["audit_source_sha256"] ==
            BASELINE_AUDIT_SHA["G3_accepted"], "G3 audit binding")
    require(records["G4"]["audit_source_sha256"] ==
            BASELINE_AUDIT_SHA["G4_accepted"], "G4 audit binding")
    require(records["G3"]["audit_test_source_sha256"] ==
            BASELINE_AUDIT_TEST_SHA["G3_accepted"], "G3 test binding")
    return records


def verify_source_tree():
    manifest = load(ACCEPTED_G4_MANIFEST)
    changed = []
    missing = []
    for relative, record in sorted(manifest["source_files"].items()):
        path = ROOT / relative
        if not path.is_file():
            missing.append(relative)
            continue
        if sha256(path) != record["sha256"]:
            changed.append(relative)
    require(not missing, "accepted source files disappeared")
    require(len(manifest["source_files"]) == manifest["source_file_count"],
            "accepted source-count self-consistency")
    require(sorted(changed) == AUTHORISED_CHANGED,
            "source changes differ from the authorised pair")
    return {
        "accepted_source_file_count": manifest["source_file_count"],
        "changed_source_files": sorted(changed),
        "missing_source_files": missing,
        "unchanged_source_file_count":
            manifest["source_file_count"] - len(changed),
        "authorised_changed_files": AUTHORISED_CHANGED,
        "current_sha256": {
            relative: sha256(ROOT / relative)
            for relative in AUTHORISED_CHANGED},
        "accepted_sha256": {
            relative: manifest["source_files"][relative]["sha256"]
            for relative in AUTHORISED_CHANGED},
    }


def verify_historical():
    results = []
    total_files = 0
    total_bytes = 0
    for run_dir, expected_manifest_sha, expected_count in HISTORICAL:
        manifest_path = run_dir / "source_manifest.json"
        require(sha256(manifest_path) == expected_manifest_sha,
                "historical manifest changed: " + run_dir.name)
        manifest = load(manifest_path)
        require(manifest["run_id"] == run_dir.name, "historical run id")
        require(manifest["run_file_count"] == expected_count,
                "historical run file count")
        excluded = set(manifest["excluded_self_referential"])
        checked = 0
        checked_bytes = 0
        for relative, record in sorted(manifest["run_files"].items()):
            if relative in excluded:
                continue
            path = run_dir / relative
            require(path.is_file(), "historical file missing: " + relative)
            require(path.stat().st_size == record["bytes"],
                    "historical file size changed: " + relative)
            require(sha256(path) == record["sha256"],
                    "historical file digest changed: " + relative)
            checked += 1
            checked_bytes += record["bytes"]
        require(checked == manifest["run_file_count"],
                "historical checked-file count")
        results.append({
            "run": str(run_dir.relative_to(ROOT)),
            "manifest_sha256": expected_manifest_sha,
            "run_file_count": manifest["run_file_count"],
            "checked_files": checked,
            "checked_bytes": checked_bytes,
            "excluded_self_referential": sorted(excluded),
            "status": "UNCHANGED",
        })
        total_files += checked
        total_bytes += checked_bytes
    return {
        "runs": results,
        "total_checked_files": total_files,
        "total_checked_bytes": total_bytes,
        "note": (
            "Old runs are read-only historical evidence.  Each is checked "
            "against its own manifest; unmanifested older runs are not "
            "individually hash-verified."),
    }


def git_state():
    def git(*args):
        return subprocess.run(
            ["git"] + list(args), cwd=str(ROOT), check=True,
            capture_output=True, text=True).stdout

    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    return {
        "head": head,
        "branch": branch,
        "head_is_baseline": head == BASELINE_HEAD,
        "staged_bytes": len(staged.encode("utf-8")),
        "status_short": git("status", "--short").splitlines(),
        "committed": "NO",
        "pushed": "NO",
        "branch_created": "NO",
    }


def main():
    records = verify_acceptance()
    source_tree = verify_source_tree()
    historical = verify_historical()
    state = git_state()
    require(state["head_is_baseline"], "HEAD changed")
    require(state["branch"] == "feature/virtual-rigid-formation-a",
            "branch changed")
    require(state["staged_bytes"] == 0, "staged changes present")
    print(json.dumps({
        "schema": "virtual_rigid_a.g3.protection_check.v1",
        "status": "PASS",
        "run_id": RUN.name,
        "acceptance": records,
        "source_tree": source_tree,
        "historical": historical,
        "git": state,
        "note": (
            "G3/G4 acceptance records bind the pre-fix source hashes and are "
            "left untouched.  They are not claimed to accept this patch; "
            "Codex must re-verify applicability on the new source."),
    }, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()