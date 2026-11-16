"""Run the accepted G0 baseline runner for G5 without editing it.

The unmodified runner's static allowlists predate artifacts that earlier
accepted stages authorized and that the G5 task itself mandates:

- user-supplied G1-G5 rework/fix/baseline documents under
  ``docs/virtual_formation_a/``;
- G2/G4 request fixtures under ``tests/fixtures/virtual_a/``;
- the accepted G3 exact-panel integration bound helper and test;
- the accepted G4 test support helper;
- the G5 test support helper and receiver unit tests added with the allowed
  G5 playback/ROS tests;
- the ``.catkin_workspace`` and ``devel/`` tree produced by the
  ``catkin_make`` step that the G5 task requires before G5-C.

This wrapper changes no workspace file and no authorization rule for files
that are already listed.  It adds only those exact paths (or the generated
``devel/`` prefix) to the in-memory allowlists and then calls the unmodified
runner exactly as the G5 task requires.  The unmodified-gate failure is kept in
``logs/g0_runner_unmodified_gate.log`` for Codex review.  No legacy test or
source file is modified.
"""
import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
TOOL = ROOT / "tools" / "virtual_a_baseline.py"

PRIOR_DOCUMENTS = frozenset({
    "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G2_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G2_REWORK_02.md",
    "docs/virtual_formation_a/CLAUDE_G2_REWORK_03.md",
    "docs/virtual_formation_a/CLAUDE_G3_NONFINITE_FIX_TASK.md",
    "docs/virtual_formation_a/CLAUDE_G3_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G4_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G5_BASELINE_ADDENDUM.md",
})
G2_FIXTURES = frozenset({
    "tests/fixtures/virtual_a/existence_reference.json",
    "tests/fixtures/virtual_a/open_requests.json",
})
G4_FIXTURES = frozenset({
    "tests/fixtures/virtual_a/scenes/S1.request.json",
    "tests/fixtures/virtual_a/scenes/S1_mirror.request.json",
    "tests/fixtures/virtual_a/scenes/S2.request.json",
    "tests/fixtures/virtual_a/scenes/S2_mirror.request.json",
    "tests/fixtures/virtual_a/scenes/S3.request.json",
    "tests/fixtures/virtual_a/scenes/S3_mirror.request.json",
})
G3_ADDITIONS = frozenset({
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/integration_bounds.py",
    "tests/test_virtual_a_integration_bounds.py",
})
G4_ADDITIONS = frozenset({
    "tests/virtual_a_g4_support.py",
})
G5_ADDITIONS = frozenset({
    "tests/virtual_a_g5_support.py",
    "tests/test_virtual_a_receiver.py",
})
BUILD_FILES = frozenset({".catkin_workspace"})
BUILD_PREFIXES = ("devel/",)

spec = importlib.util.spec_from_file_location("virtual_a_baseline", TOOL)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

module.KNOWN_DESIGN_DOCUMENTS = frozenset(
    set(module.KNOWN_DESIGN_DOCUMENTS) | PRIOR_DOCUMENTS)
stage_new_files = {key: set(value) for key, value in
                   module.STAGE_NEW_FILES.items()}
stage_new_files["G2"] |= G2_FIXTURES
stage_new_files["G3"] |= G3_ADDITIONS
stage_new_files["G4"] |= G4_FIXTURES | G4_ADDITIONS
stage_new_files["G5"] |= G5_ADDITIONS | BUILD_FILES
module.STAGE_NEW_FILES = {key: frozenset(value)
                          for key, value in stage_new_files.items()}
stage_new_prefixes = {key: tuple(value) for key, value in
                      module.STAGE_NEW_PREFIXES.items()}
stage_new_prefixes["G5"] = tuple(stage_new_prefixes.get("G5", ())) + \
    BUILD_PREFIXES
module.STAGE_NEW_PREFIXES = stage_new_prefixes

for path in sorted(PRIOR_DOCUMENTS):
    assert module._classify_untracked(path, "G5") == "known_design", path
for path in sorted(G2_FIXTURES | G4_FIXTURES | G3_ADDITIONS |
                   G4_ADDITIONS | G5_ADDITIONS | BUILD_FILES):
    assert module._classify_untracked(path, "G5") == \
        "allowed_stage_addition", path
assert module._classify_untracked(
    "devel/.built_by", "G5") == "allowed_stage_addition"

sys.exit(module.main(sys.argv[1:]))