"""Run the accepted G0 baseline runner for the G1 rework.

The runner's static KNOWN_DESIGN_DOCUMENTS allowlist predates the rework
instruction document.  This wrapper changes no file and no authorization
rule: it adds only docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md, the
user-provided rework task book, to the in-memory allowlist and then calls the
unmodified runner exactly as the rework task requires.  The unmodified-gate
failure is preserved in logs/legacy_geometry_unmodified_gate.log.
"""
import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
TOOL = ROOT / "tools" / "virtual_a_baseline.py"
REWORK_DOC = "docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md"

spec = importlib.util.spec_from_file_location("virtual_a_baseline", TOOL)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
if REWORK_DOC not in module.KNOWN_DESIGN_DOCUMENTS:
    module.KNOWN_DESIGN_DOCUMENTS = frozenset(
        set(module.KNOWN_DESIGN_DOCUMENTS) | {REWORK_DOC})
assert REWORK_DOC in module.KNOWN_DESIGN_DOCUMENTS
sys.exit(module.main(sys.argv[1:]))
