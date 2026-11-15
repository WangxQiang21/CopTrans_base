"""FR03-R1 rework baseline verification.

Run from the worktree root:

    python3 evaluation/formation_refactor_v02/FR03/rework_01/verify_baseline.py

* The ten accepted FR01/FR02 source/test files and the three frozen
  documents must still match the SHA-256 values recorded by Codex.
* The two FR03 files changed by this rework are compared against the
  snapshot hashes saved in ``pre_rework_hashes.json`` *before* editing.
* Codex's original failing probe JSON/log is hashed to show it was preserved
  unmodified while a verbatim copy of the probe was re-run here.
"""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OUT = HERE / "baseline_verification.json"
PRE = HERE / "pre_rework_hashes.json"

BASELINE = {
    # FR01 accepted (CODEX_ACCEPTANCE.json reviewed_source_sha256)
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py":
        "1df4140684c155ea5dc5533c13278c8e4cf8e590b49e77b0360194695aa333b2",
    "tests/test_uniform_motion.py":
        "7c25e85b14d8c11ecb460daf76eead60d82270dd9ad74b0d2531f490b98f360a",
    # FR02-A accepted (CODEX_ACCEPTANCE.json / evidence_manifest.json)
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_trajectory.py":
        "a93c2b2faba8bda7ac4592888cd502ea3850e6138505df851a12f44a5e86b579",
    "tests/test_uniform_trajectory.py":
        "713374e57559a1388d5940bf1fcc1aff6e255dd32644a6cede293272f08a03c2",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_timing.py":
        "91678441d589e48e5bb81c688bce8518df4bc2d4119a7de749fff3ba4dd9af18",
    "tests/test_uniform_timing.py":
        "7fc4bf7f509b5860f99f78f761c0ea03402ab4cc182d802d262ecdd77882de44",
    # FR02-B accepted (CODEX_ACCEPTANCE.json reviewed_source_sha256)
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_collision.py":
        "31ea1831b0fd4fe05ff9292e560ea11d8b7392787fba3f4971d157c961fd5fc5",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_quality.py":
        "95830366a57639cfb47329654fcaa2042dd252bb9a15c12b45178d203a6c7511",
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_quality.py":
        "7164decbf79beb812d075146eae9c0b936efa00aa7d0a3b73c16ac91492efa96",
    "tests/test_uniform_quality.py":
        "881922fdbc9d901130813e80848661608ed00210d0fb7daedf78b091a9fb47ae",
}

FROZEN = {
    # FR02-A baseline/input_verification.json expected_sha256
    "docs/FORMATION_REFACTOR_DECISIONS.md":
        "8f2af8b8c21c6f86a9dba467bfc4a9297229998fbaefba52ad5487eabf2f90a9",
    "docs/FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md":
        "88d190b9a14aa6b95987c211d1bfa2c170bc9c26b9d78ecc8da4c052a4c46d93",
    "docs/CLAUDE_TASK_01.md":
        "bea179cb661775a80283c0923606499da9ee2b9d0010256ff30eac4eb90cb07b",
}

REWORKED = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py",
    "tests/test_uniform_search.py",
)

PRE_FR03_EVIDENCE = (
    "evaluation/formation_refactor_v02/FR03/successors.json",
    "evaluation/formation_refactor_v02/FR03/state_key_samples.json",
    "evaluation/formation_refactor_v02/FR03/arc_step_audit.json",
    "evaluation/formation_refactor_v02/FR03/successor_fr02_audit.json",
    "evaluation/formation_refactor_v02/FR03/measure_200_states.json",
)

CODEX_PROBE_FILES = (
    "evaluation/formation_refactor_v02/FR03/codex_acceptance_20261113T164549Z/"
    "key_compatibility_probe.json",
    "evaluation/formation_refactor_v02/FR03/codex_acceptance_20261113T164549Z/"
    "key_compatibility_probe.log",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    payload = {
        "schema": "fr03.rework_01.baseline_verification.v1",
        "baseline_sources": {},
        "frozen_documents": {},
        "reworked_files": {},
        "pre_fr03_evidence": {},
        "codex_probe_originals": {},
        "all_accepted_match": True,
        "all_frozen_match": True,
        "unexpected_changes": [],
        "pre_rework_snapshot_found": PRE.is_file(),
    }
    for path, expected in BASELINE.items():
        actual = sha256(ROOT / path)
        match = actual == expected
        payload["baseline_sources"][path] = {
            "expected_sha256": expected,
            "actual_sha256": actual,
            "match": match,
        }
        if not match:
            payload["all_accepted_match"] = False
            payload["unexpected_changes"].append(path)
    for path, expected in FROZEN.items():
        actual = sha256(ROOT / path)
        match = actual == expected
        payload["frozen_documents"][path] = {
            "expected_sha256": expected,
            "actual_sha256": actual,
            "match": match,
        }
        if not match:
            payload["all_frozen_match"] = False
            payload["unexpected_changes"].append(path)

    pre = json.loads(PRE.read_text()) if PRE.is_file() else {}
    pre_files = pre.get("files", {})
    snapshot_prefix = (
        "evaluation/formation_refactor_v02/FR03/rework_01/pre_rework/")
    for path in REWORKED:
        snapshot = snapshot_prefix + Path(path).name
        before = pre_files.get(snapshot, {})
        after = sha256(ROOT / path)
        payload["reworked_files"][path] = {
            "pre_rework_sha256": before.get("sha256"),
            "pre_rework_bytes": before.get("bytes"),
            "snapshot_copy": snapshot,
            "post_rework_bytes": (ROOT / path).stat().st_size,
            "post_rework_sha256": after,
            "changed_by_rework": before.get("sha256") != after,
        }
    # The pre-rework FR03 evidence at the FR03 top level must still be the
    # exact bytes written before this rework.
    for path in PRE_FR03_EVIDENCE:
        digest = sha256(ROOT / path)
        snapshot = snapshot_prefix + "evidence/" + Path(path).name
        expected = pre_files.get(snapshot, {}).get("sha256")
        payload["pre_fr03_evidence"][path] = {
            "sha256": digest,
            "snapshot_sha256": expected,
            "matches_snapshot": expected == digest,
        }
    # Codex's failing probe artifacts are preserved verbatim.
    for path in CODEX_PROBE_FILES:
        full = ROOT / path
        if full.is_file():
            payload["codex_probe_originals"][path] = {
                "bytes": full.stat().st_size,
                "sha256": sha256(full),
                "preserved": True,
            }
    OUT.write_text(json.dumps(payload, indent=1) + "\n")
    print(json.dumps({
        "all_accepted_match": payload["all_accepted_match"],
        "all_frozen_match": payload["all_frozen_match"],
        "unexpected_changes": payload["unexpected_changes"],
        "reworked_changed": {
            path: value["changed_by_rework"]
            for path, value in payload["reworked_files"].items()},
        "pre_rework_snapshot_found": payload["pre_rework_snapshot_found"],
    }, indent=1))


if __name__ == "__main__":
    main()