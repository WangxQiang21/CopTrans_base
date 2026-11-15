"""FR04 baseline verification: accepted sources and frozen docs are untouched.

Run from the worktree root:

    python3 -I evaluation/formation_refactor_v02/FR04/verify_baseline.py

Expected hashes are copied verbatim from the independent Codex acceptance
records (FR01, FR02-A, FR02-B, FR03-R1).  FR04 is allowed to modify
``uniform_search.py`` only by (a) adding``import heapq``/``import time`` and
(b) appending the FR04 search section; the FR03-accepted bytes must still be
recoverable by removing exactly those additions.  ``uniform_pipeline.py`` and
``tests/test_uniform_pipeline.py`` are the only other new FR04 files.
"""
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / "baseline_verification.json"

BASELINE = {
    # FR01 accepted (CODEX_ACCEPTANCE.json reviewed_source_sha256)
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py":
        "1df4140684c155ea5dc5533c13278c8e4cf8e590b49e77b0360194695aa333b2",
    "tests/test_uniform_motion.py":
        "7c25e85b14d8c11ecb460daf76eead60d82270dd9ad74b0d2531f490b98f360a",
    # FR02-A accepted (CODEX_ACCEPTANCE.json reviewed_source_sha256)
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
    # FR03-R1 accepted (CODEX_ACCEPTANCE.json reviewed_source_sha256)
    "tests/test_uniform_search.py":
        "87f3f032dc3732e59dd1c19282118d8ddbfb727a4d049609316ca6b462f997e5",
}

FROZEN = {
    "docs/FORMATION_REFACTOR_DECISIONS.md":
        "8f2af8b8c21c6f86a9dba467bfc4a9297229998fbaefba52ad5487eabf2f90a9",
    "docs/FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md":
        "88d190b9a14aa6b95987c211d1bfa2c170bc9c26b9d78ecc8da4c052a4c46d93",
    "docs/CLAUDE_TASK_01.md":
        "bea179cb661775a80283c0923606499da9ee2b9d0010256ff30eac4eb90cb07b",
}

UNIFORM_SEARCH = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py")
FR03_UNIFORM_SEARCH = (
    "982b2f96e1ab6dfcb3008d1e7232432bd861ebe8c8b9376e65bfbd96f8f96e83")
FR04_MARKER = (b"# ---------------------------------------------------------------------------\n"
               b"# FR04: budgeted autonomous graph search")
FR04_IMPORTS = (b"import heapq\n", b"import time\n")

FR04_NEW = (
    UNIFORM_SEARCH,
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_pipeline.py",
    "tests/test_uniform_pipeline.py",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fr03_prefix_check(path):
    """The FR03 accepted bytes must be recoverable from the FR04 file."""
    data = Path(path).read_bytes()
    parts = data.split(FR04_MARKER)
    if len(parts) != 2:
        return {"match": False, "detail": "exactly one FR04 marker expected",
                "marker_count": len(parts) - 1}
    prefix = parts[0]
    import_counts = {}
    for token in FR04_IMPORTS:
        count = prefix.count(token)
        import_counts[token.decode().strip()] = count
        if count < 1:
            return {"match": False,
                    "detail": "expected at least one {!r}".format(token),
                    "import_counts": import_counts}
        # ``import time`` also appears once inside an FR03 function; only the
        # first (top-of-file) occurrence is the FR04 addition.
        prefix = prefix.replace(token, b"", 1)
    import_counts["retained_after_removal"] = {
        token.decode().strip(): prefix.count(token)
        for token in FR04_IMPORTS}
    # The accepted FR03 file ended with one newline; the FR04 append left two
    # further blank lines before its section marker.  The FR03 accepted bytes
    # must be an exact prefix of the current pre-marker region.
    accepted = prefix.rstrip() + b"\n"
    actual = hashlib.sha256(accepted).hexdigest()
    return {
        "match": actual == FR03_UNIFORM_SEARCH and prefix.startswith(accepted),
        "expected_sha256": FR03_UNIFORM_SEARCH,
        "reconstructed_sha256": actual,
        "fr03_prefix_is_exact_prefix_of_current": prefix.startswith(accepted),
        "extra_blank_lines_before_marker": len(prefix) - len(accepted),
        "import_counts": import_counts,
        "appended_bytes": len(data) - len(parts[0]),
        "appended_lines": data[len(parts[0]):].count(b"\n"),
    }


def main():
    payload = {
        "schema": "fr04.baseline_verification.v1",
        "baseline_sources": {},
        "frozen_documents": {},
        "fr03_source_append_check": {},
        "fr04_new_files": {},
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
    payload["fr03_source_append_check"] = fr03_prefix_check(ROOT / UNIFORM_SEARCH)
    for path in FR04_NEW:
        payload["fr04_new_files"][path] = {
            "bytes": (ROOT / path).stat().st_size,
            "sha256": sha256(ROOT / path),
        }
    OUT.write_text(json.dumps(payload, indent=1) + "\n")
    print(json.dumps({
        "all_accepted_match": payload["all_accepted_match"],
        "all_frozen_match": payload["all_frozen_match"],
        "fr03_prefix_match": payload["fr03_source_append_check"]["match"],
        "unexpected_changes": payload["unexpected_changes"],
    }, indent=1))


if __name__ == "__main__":
    main()