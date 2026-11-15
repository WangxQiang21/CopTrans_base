"""FR03 baseline verification: accepted sources and frozen docs are untouched.

Run from the worktree root:

    python3 evaluation/formation_refactor_v02/FR03/verify_baseline.py

Expected hashes are copied verbatim from the independent Codex acceptance
records (FR01, FR02-A, FR02-B) and the FR02-A frozen-document input
verification.  Only the two FR03 files are allowed to be new.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / "baseline_verification.json"

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

FR03_NEW = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py",
    "tests/test_uniform_search.py",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    payload = {
        "schema": "fr03.baseline_verification.v1",
        "baseline_sources": {},
        "frozen_documents": {},
        "fr03_new_files": {},
        "all_accepted_match": True,
        "all_frozen_match": True,
        "unexpected_changes": [],
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
    for path in FR03_NEW:
        payload["fr03_new_files"][path] = {
            "bytes": (ROOT / path).stat().st_size,
            "sha256": sha256(ROOT / path),
        }
    OUT.write_text(json.dumps(payload, indent=1) + "\n")
    print(json.dumps({
        "all_accepted_match": payload["all_accepted_match"],
        "all_frozen_match": payload["all_frozen_match"],
        "unexpected_changes": payload["unexpected_changes"],
    }, indent=1))


if __name__ == "__main__":
    main()