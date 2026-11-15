"""Assert the expected phase-boundary outcomes of the copied Codex probe.

The probe inputs and the probe itself are kept verbatim from
``codex_acceptance_20261114T113812Z/feedback_phase_boundary_probe.py``; this
checker only reads the JSON it produced and asserts the FR05-R2 contract.
"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
rows = {row["case"]: row for row in
        json.loads((HERE / "feedback_phase_boundary_probe.json").read_text())}

# case -> (final_state, playback_seen, number of published events)
EXPECTED = {
    # published, wrong digest: still ignored
    "original_wrong_digest": ("PLAYING", False, 0),
    # published, matching digest: terminal accepted
    "published_matching_digest": ("HOLD_END", True, 2),
    # not published, digest missing/null: must not be treated as a match
    "prepublication_missing_digest": ("PLANNING", False, 0),
    "prepublication_null_digest": ("PLANNING", False, 0),
    "prepublication_missing_start_digest": ("PLANNING", False, 0),
}

assert set(rows) == set(EXPECTED), (set(rows) ^ set(EXPECTED))
failures = []
for case, (state, seen, events) in EXPECTED.items():
    row = rows[case]
    actual = (row["final_state"], row["playback_seen"], len(row["events"]))
    print("%-36s final=%-8s seen=%-5s events=%d %s" % (
        case, row["final_state"], row["playback_seen"], len(row["events"]),
        "ok" if actual == (state, seen, events) else "MISMATCH"))
    if actual != (state, seen, events):
        failures.append((case, actual, (state, seen, events)))

assert not failures, failures
print("PROBE_ASSERTIONS_OK")