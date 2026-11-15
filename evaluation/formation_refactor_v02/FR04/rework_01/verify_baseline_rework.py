"""Run FR04's baseline verifier without overwriting its original result.

The accepted verifier writes ``FR04/baseline_verification.json``.  The FR04-R1
rework must preserve all earlier evidence, so this wrapper loads the same
module and points only its output path into ``rework_01/``.
"""
import importlib.util
from pathlib import Path

ROOT = Path.cwd()
SOURCE = (ROOT / "evaluation" / "formation_refactor_v02" / "FR04"
          / "verify_baseline.py")
OUT = Path(__file__).resolve().parent / "baseline_verification.json"

spec = importlib.util.spec_from_file_location("fr04_verify_baseline", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.OUT = OUT
module.main()