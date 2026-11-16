"""G0 rework targeted probes for R1 (stage authorization) and R2 (runner guard).

Pure in-memory manifests and mocked children/writes.  No old test case, FR06
search, ROS launch or historical directory is touched.
"""
from pathlib import Path
from unittest.mock import patch
import hashlib
import importlib.util
import json
import shutil
import sys

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
TOOL = ROOT / "tools/virtual_a_baseline.py"
INTERFACE = ROOT / "docs/virtual_formation_a/G0_INTERFACE_CHECK.md"
ORIGINAL_RUN = ROOT / "evaluation/virtual_formation_a/G0/20261115T140936Z"
CODEX_REVIEW = ROOT / "evaluation/virtual_formation_a/G0/codex_review_20261115T141437Z"
P = "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"

spec = importlib.util.spec_from_file_location("rework_va", TOOL)
va = importlib.util.module_from_spec(spec)
spec.loader.exec_module(va)


def save(name, value):
    path = OUT / name
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


real = va.collect_manifest(ROOT, va.DEFAULT_PROTECTED_PATHS)


def manifest_with(tracked=(), untracked_unknown=()):
    manifest = dict(real)
    manifest["git"] = dict(real["git"])
    manifest["git"]["tracked_changes"] = list(tracked)
    manifest["untracked"] = dict(real["untracked"])
    manifest["untracked"]["unknown"] = list(untracked_unknown)
    # Caller claims these are already resolved; the gate must ignore the claim.
    manifest["unknown_modifications"] = []
    return manifest


def gate(level, tracked=(), untracked_unknown=()):
    try:
        va._workspace_gate(ROOT, manifest_with(tracked, untracked_unknown),
                           level=level)
    except va.BaselineError as exc:
        return "REJECT", str(exc)
    return "ACCEPT", ""


# R1: stage authorization positive/negative cases.
G1_SETUP = " M catkin_ws/src/fwis_v3_optimizer/setup.py"
G5_NODE = (" M catkin_ws/src/fwis_v3_optimizer/scripts/"
           "formation_uniform_planner_node.py")
CASES = [
    ("g1_setup_edit_allowed_at_geometry", "geometry", (G1_SETUP,), (), "ACCEPT"),
    ("g1_setup_edit_rejected_at_baseline", "baseline", (G1_SETUP,), (), "REJECT"),
    ("g1_new_package_allowed_at_geometry", "geometry", (),
     (P + "formation_continuous_a/geometry.py",), "ACCEPT"),
    ("g1_new_package_rejected_at_baseline", "baseline", (),
     (P + "formation_continuous_a/geometry.py",), "REJECT"),
    ("g2_optimizer_rejected_at_geometry", "geometry", (),
     (P + "formation_continuous_a/optimizer.py",), "REJECT"),
    ("g2_optimizer_allowed_at_timing", "timing", (),
     (P + "formation_continuous_a/optimizer.py",), "ACCEPT"),
    ("vehicle_yaml_rejected_at_full", "full",
     (" M catkin_ws/src/fwis_v3_demo/config/transport_formation.yaml",), (),
     "REJECT"),
    ("old_algorithm_rejected_at_full", "full",
     (" M " + P + "formation.py",), (), "REJECT"),
    ("old_test_rejected_at_full", "full",
     (" M tests/test_uniform_motion.py",), (), "REJECT"),
    ("deletion_rejected_at_full", "full",
     (" D " + P + "formation.py",), (), "REJECT"),
    ("g5_node_allowed_at_full", "full", (G5_NODE,), (), "ACCEPT"),
    ("g5_node_rejected_at_timing", "timing", (G5_NODE,), (), "REJECT"),
    ("g4_fixture_map_allowed_at_full", "full", (),
     ("tests/fixtures/virtual_a/maps/x.pgm",), "ACCEPT"),
    ("g4_fixture_map_rejected_at_geometry", "geometry", (),
     ("tests/fixtures/virtual_a/maps/x.pgm",), "REJECT"),
    ("unknown_file_rejected_at_full", "full", (),
     ("tests/test_not_authorized.py",), "REJECT"),
]
stage_results = []
for name, level, tracked, untracked, expected in CASES:
    actual, detail = gate(level, tracked, untracked)
    stage_results.append({
        "case": name,
        "level": level,
        "workspace_stage": va.REGRESSION_STAGES[level],
        "expected": expected,
        "actual": actual,
        "detail": detail,
        "pass": actual == expected,
    })
assert all(row["pass"] for row in stage_results), stage_results
save("stage_gate_cases.json", {
    "schema": "virtual_rigid_a.g0.rework.stage_gate.v1",
    "method": "in-memory manifest simulation; real git status unmodified",
    "real_unknown_modifications": va._stage_unknown_modifications(real, "G0"),
    "cases": stage_results,
})


# R2: containment/no-overwrite preflight must reject before any side effect.
def guarded_call(target, level):
    side_effects = []
    fake = type("Fake", (), {"returncode": 0, "stdout": "MOCK", "stderr": ""})()
    with patch.object(va.subprocess, "run",
                      side_effect=lambda *a, **k: side_effects.append("subprocess")), \
            patch.object(va, "_write_text",
                         side_effect=lambda *a, **k: side_effects.append("write")):
        try:
            va.run_regression(ROOT, target, level)
        except va.BaselineError as exc:
            return "REJECT", str(exc), side_effects
        return "ACCEPT", "", side_effects


probe_root = OUT / "guard_probe"
probe_root.mkdir()
guard_results = []
for name, target in (
        ("historical_fr06", ROOT / "evaluation/formation_refactor_v02/FR06"),
        ("external_tmp", Path("/tmp/virtual_a_rework_guard_probe")),
        ("evidence_root", ROOT / "evaluation/virtual_formation_a")):
    status, detail, effects = guarded_call(target, "geometry")
    guard_results.append({
        "case": name, "target": str(target), "expected": "REJECT",
        "actual": status, "detail": detail,
        "subprocess_starts": 0, "writes": 0,
        "pass": status == "REJECT" and not effects,
    })
existing = probe_root / "existing_case_output"
(existing / "regression_artifacts" / "geometry_continuous").mkdir(parents=True)
status, detail, effects = guarded_call(existing, "geometry")
guard_results.append({
    "case": "existing_case_artifact", "target": str(existing),
    "expected": "REJECT", "actual": status, "detail": detail,
    "subprocess_starts": 0, "writes": 0,
    "pass": status == "REJECT" and not effects,
})
# A clean run directory that already holds main's metadata is still accepted.
clean = probe_root / "clean_run_with_metadata"
clean.mkdir()
(clean / "results.json").write_text("{}\n", encoding="utf-8")
(clean / "protected_before.json").write_text("{}\n", encoding="utf-8")
(clean / "logs").mkdir()
(clean / "logs" / "g0_summary.log").write_text("metadata\n", encoding="utf-8")
fake = type("Fake", (), {"returncode": 0, "stdout": "MOCK", "stderr": ""})()
with patch.object(va.subprocess, "run", return_value=fake) as child, \
        patch.object(va, "_write_text", side_effect=lambda *a, **k: None):
    rows = va.run_regression(ROOT, clean, "geometry")
guard_results.append({
    "case": "clean_run_with_main_metadata", "target": str(clean),
    "expected": "ACCEPT", "actual": "ACCEPT",
    "mock_child_starts": child.call_count, "rows": len(rows),
    "pass": (child.call_count == len(va.REGRESSION_LEVELS["geometry"])
             and len(rows) == len(va.REGRESSION_LEVELS["geometry"])),
    "note": "one mocked child per case; main's metadata files are not case paths",
})
assert all(row["pass"] for row in guard_results), guard_results
save("runner_guard_cases.json", {
    "schema": "virtual_rigid_a.g0.rework.runner_guard.v1",
    "method": "mocked subprocess.run and _write_text; zero real child starts",
    "cases": guard_results,
})

# The submitted original run must remain byte-identical.  The tool and the
# interface note are intentionally modified by this rework and are listed
# separately, not counted as evidence drift.
submission = json.loads((CODEX_REVIEW / "submission_before.json").read_text())
RUN_PREFIX = str(ORIGINAL_RUN.relative_to(ROOT)) + "/"
run_entries = {path: digest for path, digest in submission.items()
               if path.startswith(RUN_PREFIX)}
rework_entries = {path: digest for path, digest in submission.items()
                  if not path.startswith(RUN_PREFIX)}
assert set(rework_entries) == {
    "tools/virtual_a_baseline.py",
    "docs/virtual_formation_a/G0_INTERFACE_CHECK.md"}, sorted(rework_entries)
run_drift = sorted(path for path, digest in run_entries.items()
                   if sha(ROOT / path) != digest)
current = {path: sha(ROOT / path) for path in rework_entries}
save("original_evidence_preserved.json", {
    "original_run": str(ORIGINAL_RUN.relative_to(ROOT)),
    "checked_run_files": len(run_entries),
    "run_drift": run_drift,
    "run_pass": not run_drift,
    "intentional_rework_files": {
        path: {"before": rework_entries[path], "after": current[path]}
        for path in sorted(rework_entries)},
    "codex_review_dir": str(CODEX_REVIEW.relative_to(ROOT)),
    "pass": not run_drift,
})

save("hashes.json", {
    "tools/virtual_a_baseline.py": sha(TOOL),
    "docs/virtual_formation_a/G0_INTERFACE_CHECK.md": sha(INTERFACE),
    "reviewed_by_codex_previous_tool_sha256":
        "77d280314198debca090dea9472df69f7a0d9b0817e88b2fd511234b38ba8dcb",
    "reviewed_by_codex_previous_interface_sha256":
        "97c5c411eead4c21abfbd2c70aa6c1af338ef2ce69f8ca6df9f9ef540b8917d3",
})

print(json.dumps({
    "stage_gate_cases": len(stage_results),
    "stage_gate_all_pass": all(row["pass"] for row in stage_results),
    "runner_guard_cases": len(guard_results),
    "runner_guard_all_pass": all(row["pass"] for row in guard_results),
    "original_evidence_checked": len(run_entries),
    "original_evidence_drift": run_drift,
    "hashes": {
        "tool": sha(TOOL),
        "interface": sha(INTERFACE),
    },
}, indent=2, sort_keys=True))