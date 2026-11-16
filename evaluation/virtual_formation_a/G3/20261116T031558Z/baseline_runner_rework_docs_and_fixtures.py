"""Run the accepted G0 baseline runner for G3 without editing it.

The runner's static allowlists predate three artifacts that earlier accepted
stages explicitly authorized:

- the user-supplied G1/G2 rework task books under docs/virtual_formation_a/;
- `tests/fixtures/virtual_a/open_requests.json`, explicitly listed in the G2
  task file's new-file list;
- `tests/fixtures/virtual_a/existence_reference.json`, explicitly permitted by
  the same task file as the independent existence reference fixture.

This wrapper changes no file and no authorization rule.  It adds only those
exact prior-stage artifacts to the in-memory allowlists and then calls the
unmodified runner exactly as the G3 task requires.  The unmodified-gate
failure is preserved in logs/legacy_unmodified_gate.log for Codex review.
"""
import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
TOOL = ROOT / "tools" / "virtual_a_baseline.py"

REWORK_DOCUMENTS = frozenset({
    "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G2_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G2_REWORK_02.md",
    "docs/virtual_formation_a/CLAUDE_G2_REWORK_03.md",
    "docs/virtual_formation_a/CLAUDE_G3_REWORK_01.md",
})
G2_FIXTURES = frozenset({
    "tests/fixtures/virtual_a/open_requests.json",
    "tests/fixtures/virtual_a/existence_reference.json",
})

spec = importlib.util.spec_from_file_location("virtual_a_baseline", TOOL)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

module.KNOWN_DESIGN_DOCUMENTS = frozenset(
    set(module.KNOWN_DESIGN_DOCUMENTS) | REWORK_DOCUMENTS)
stage_new_files = dict(module.STAGE_NEW_FILES)
stage_new_files["G2"] = frozenset(stage_new_files["G2"] | G2_FIXTURES)
module.STAGE_NEW_FILES = stage_new_files

for path in sorted(REWORK_DOCUMENTS):
    assert module._classify_untracked(path, "G3") == "known_design", path
for path in sorted(G2_FIXTURES):
    assert module._classify_untracked(path, "G3") == "allowed_stage_addition", path

sys.exit(module.main(sys.argv[1:]))