"""Independent verification of the G3 rework source-and-evidence manifest.

Run with ``python3 -B -I``.  This script uses only the Python standard library
and does not import the project.  It recomputes file hashes, reconstructs the
Codex-accepted G2 protected tree from the accepted G1 snapshot and the G2 diff,
rechecks the G3 whitelist, and revalidates the benchmark/model/test/legacy
records plus the independent interval-coverage witnesses from their raw files.
It never writes the manifest and never claims Codex acceptance.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
MANIFEST = RUN / "source_manifest.json"
SCHEMA = "virtual_rigid_a.g3.rework.source_manifest.v1"
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
G1_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G1/20261115T152858Z"
G1_PROTECTED = G1_RUN / "legacy/protected_after.json"
G2_ACCEPTANCE = (REPO_ROOT / "evaluation/virtual_formation_a/G2" /
                 "codex_acceptance_rework_03/CODEX_ACCEPTANCE.json")
G2_RUN = REPO_ROOT / "evaluation/virtual_formation_a/G2/20261115T174526Z"
G2_MANIFEST = G2_RUN / "source_manifest.json"
G2_PROTECTION = G2_RUN / "analysis/protection_diff_vs_g1_accepted.json"
EXPECTED_G1_TREE = (
    "910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519")
EXPECTED_G1_AFTER_SHA = (
    "c9e4c874c3e7d0dc05a1850d74febd28ec7475a232b6f8f98d3e1c11d2892470")
EXPECTED_G2_MANIFEST_SHA = (
    "852cb44615321ed69613f6dfbe6ae6b2b79da5a17563c38537e899209dc90886")
EXPECTED_G2_TREE = (
    "fa1b0596f7b9ecef616512785ab65ef2b93884dfde3a9c2a9794a9f7ac222f67")
PACKAGE = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
           "formation_continuous_a/")
G3_SOURCES = sorted([
    PACKAGE + "timing.py",
    PACKAGE + "trajectory.py",
    "docs/virtual_formation_a/CLAUDE_G3_REWORK_01.md",
    "docs/virtual_formation_a/CLAUDE_G3_TASK.md",
    "tests/test_virtual_a_timing.py",
    "tests/test_virtual_a_trajectory.py",
])
REFERENCE_DOCUMENTS = sorted([
    "docs/VIRTUAL_FORMATION_A_FR06_TECHNICAL_REVIEW_20261115.md",
])
EXPECTED_EXCLUSIONS = [
    "source_manifest.json",
    "logs/source_manifest_generation.log",
    "logs/source_manifest_verification.log",
]
ALLOWED_NEW = sorted([
    PACKAGE + "timing.py",
    PACKAGE + "trajectory.py",
    "tests/test_virtual_a_timing.py",
    "tests/test_virtual_a_trajectory.py",
])
ALLOWED_CHANGED = sorted([
    PACKAGE + "audit.py",
    PACKAGE + "bounds.py",
    PACKAGE + "contracts.py",
    "tests/test_virtual_a_audit.py",
    "tools/virtual_a_benchmark.py",
])
EXPECTED_TEST_COUNTS = {"audit": 51, "timing": 18, "trajectory": 23}
EXPECTED_CHECK_COUNT = 151
EXPECTED_SOURCE_COUNT = 31
EXPECTED_MOTION_AUDIT_CHECKS = 909
EXPECTED_HOLD_AUDIT_CHECKS = 16
EXPECTED_CERTIFICATE_LEAVES = 64
EXPECTED_POINTWISE_VEHICLE_CHECKS = 3 * EXPECTED_CERTIFICATE_LEAVES
EXPECTED_DEGENERATE_NORM = 0.10677546534667971
NUMERICS_SCHEMA = "virtual_rigid_a.g3.rework_numerics.v1"
EXPECTED_CASES = [
    "E1", "E2", "E3", "E4", "E5", "E6", "NO_MOTION_HOLD",
    "STRAIGHT_LINE", "STRAIGHT_POLICY"]
MOTION_CASES = [name for name in EXPECTED_CASES if name != "NO_MOTION_HOLD"]
MODEL_KEYS = {
    "backend", "model_version", "numerics", "path", "path_model_version",
    "planning_context", "request_inputs", "request_sha256", "timing"}
TIMING_KEYS = {"S", "T_s", "kind", "law", "selection"}
SELECTION_KEYS = {
    "T0_s", "interval_subdivisions_per_segment", "metrics_at_T0_s",
    "required_duration_s", "rescalings", "safety_factor",
    "selected_duration_s", "wheel_angle_bound_rad",
    "wheel_angle_limit_rad", "worst"}
SCALED_METRICS = {
    "speed": ("v_max", 1),
    "acceleration": ("a_max", 2),
    "jerk": ("jerk_max", 3),
    "yaw_rate": ("yaw_rate_max", 1),
    "yaw_acceleration": ("yaw_accel_max", 2),
    "yaw_jerk": ("yaw_jerk_max", 3),
    "lateral_acceleration": ("lateral_accel_max", 2),
    "wheel_speed": ("wheel_speed_max", 1),
    "steer_rate": ("steer_rate_max", 1),
    "steer_acceleration": ("steer_accel_max", 2),
}
UTILIZATION_KEYS = set(SCALED_METRICS) | {"wheel_angle"}
DIFFERENTIAL_TOLERANCES = {
    "velocity": 1e-5, "acceleration": 1e-4, "jerk": 1e-2,
    "angular_rate": 1e-5, "angular_accel": 1e-4, "angular_jerk": 1e-2,
}
FORBIDDEN_MODEL_KEYS = {
    "model_sha256", "audit_report", "dynamic_audit", "audit",
    "codex_acceptance", "collision_checked", "executable", "map_used",
    "ros_used",
}
EXCLUDED_BASENAMES = {"virtual_a_baseline.py"}
EXCLUDED_PARTS = {"__pycache__"}
EXCLUDED_SUFFIXES = {".pyc", ".pyo"}


class Checker:
    def __init__(self):
        self.count = 0
        self.failures = []

    def check(self, condition, message):
        self.count += 1
        if not condition:
            self.failures.append(message)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_bytes(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")


def canonical_sha(value):
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def file_record(path):
    path = Path(path)
    return {"bytes": path.stat().st_size, "sha256": sha256(path)}


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def canonical_protected_tree(entries):
    payload = {
        entry["path"]: {
            "type": entry["type"],
            "sha256": entry["sha256"],
            "size_bytes": entry["size_bytes"],
        }
        for entry in entries
    }
    return canonical_sha(payload)


def current_entries(roots):
    rows = []
    for raw_root in roots:
        root = REPO_ROOT / raw_root
        if not root.exists():
            continue
        candidates = [root] if (root.is_file() or root.is_symlink()) else sorted(
            path for path in root.rglob("*")
            if path.is_file() or path.is_symlink())
        for path in candidates:
            relative = path.relative_to(REPO_ROOT).as_posix()
            parts = set(path.relative_to(REPO_ROOT).parts)
            if path.name in EXCLUDED_BASENAMES:
                continue
            if parts & EXCLUDED_PARTS:
                continue
            if path.suffix in EXCLUDED_SUFFIXES:
                continue
            if path.is_symlink():
                target_text = os.readlink(path)
                rows.append({
                    "path": relative, "type": "symlink",
                    "sha256": hashlib.sha256(
                        os.fsencode(target_text)).hexdigest(),
                    "size_bytes": 0,
                })
            else:
                rows.append({
                    "path": relative, "type": "file",
                    "sha256": sha256(path),
                    "size_bytes": path.stat().st_size,
                })
    return rows


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(REPO_ROOT), check=True,
        capture_output=True, text=True).stdout


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def check_float(checker, value, label):
    checker.check(finite(value), "not finite: " + label)
    return float(value) if finite(value) else 0.0


def recursive_keys(value, path=""):
    if isinstance(value, dict):
        for key, child in value.items():
            yield path + "/" + str(key), key, child
            for item in recursive_keys(child, path + "/" + str(key)):
                yield item
    elif isinstance(value, list):
        for index, child in enumerate(value):
            for item in recursive_keys(child, path + "[{}]".format(index)):
                yield item


def verify_test_log(checker, results, name):
    expected = EXPECTED_TEST_COUNTS[name]
    text = (RUN / "logs" / (name + ".log")).read_text(encoding="utf-8")
    match = re.search(r"Ran (\d+) tests? in ([0-9.]+)s", text)
    checker.check(match is not None,
                  "{} test log has no unittest summary".format(name))
    if match is not None:
        checker.check(int(match.group(1)) == expected,
                      "{} test count changed".format(name))
    checker.check(text.rstrip().splitlines()[-1].strip() == "OK",
                  "{} test log does not end in OK".format(name))
    record = results["tests"][name]
    checker.check(record["count"] == expected,
                  "{} results count changed".format(name))
    checker.check(record["status"] == "OK",
                  "{} results status changed".format(name))
    checker.check(record["log_sha256"] == sha256(
        RUN / "logs" / (name + ".log")),
        "{} log hash changed".format(name))


def verify_g2(checker):
    g1 = load(G1_PROTECTED)
    checker.check(sha256(G1_PROTECTED) == EXPECTED_G1_AFTER_SHA,
                  "G1 protected_after.json changed")
    checker.check(g1["protected_tree_sha256"] == EXPECTED_G1_TREE,
                  "G1 tree declaration changed")
    checker.check(canonical_protected_tree(g1["protected_files"]) ==
                  EXPECTED_G1_TREE, "G1 snapshot is inconsistent")

    acceptance = load(G2_ACCEPTANCE)
    checker.check(acceptance.get("status") == "ACCEPTED",
                  "G2 acceptance is not ACCEPTED")
    checker.check(acceptance.get("source_manifest_sha256") ==
                  EXPECTED_G2_MANIFEST_SHA,
                  "G2 acceptance manifest binding changed")
    checker.check(acceptance.get("next_stage_authorized") is False,
                  "G2 acceptance unexpectedly authorizes G3")
    checker.check(sha256(G2_MANIFEST) == EXPECTED_G2_MANIFEST_SHA,
                  "G2 source manifest changed")
    g2_manifest = load(G2_MANIFEST)
    checker.check(g2_manifest["protected"][
        "current_full_protected_tree_sha256"] == EXPECTED_G2_TREE,
        "G2 manifest tree changed")
    g2_protection = load(G2_PROTECTION)
    checker.check(g2_protection["current"][
        "full_protected_tree_sha256"] == EXPECTED_G2_TREE,
        "G2 protection tree changed")
    checker.check(g2_protection["authorization"][
        "only_authorized_changes"] is True,
        "G2 protection diff is not authorized")
    checker.check(g2_protection["removed_vs_g1_accepted"] == [],
                  "G2 recorded protected removals")
    for relative, expected in acceptance["source_sha256"].items():
        source = g2_manifest["source_files"].get(relative)
        checker.check(source is not None and source["sha256"] == expected,
                      "G2 accepted source hash unbound: " + relative)

    excluded = set(g2_manifest.get("excluded_self_referential", []))
    checked = 0
    for relative, record in sorted(g2_manifest["run_files"].items()):
        if relative in excluded:
            continue
        path = G2_RUN / relative
        checker.check(path.is_file() and sha256(path) == record["sha256"],
                      "accepted G2 run file changed: " + relative)
        checked += 1
    checker.check(checked > 0, "no G2 run files were checked")

    g2_rows = {row["path"]: dict(row) for row in
               g1["protected_files"]}
    for row in g2_protection["added_vs_g1_accepted"]:
        g2_rows[row["path"]] = {
            "path": row["path"], "type": "file",
            "sha256": row["sha256"], "size_bytes": row["size_bytes"]}
    for row in g2_protection["changed_vs_g1_accepted"]:
        old = g2_rows[row["path"]]
        g2_rows[row["path"]] = {
            "path": row["path"], "type": old["type"],
            "sha256": row["g2_sha256"],
            "size_bytes": row["g2_size_bytes"]}
    checker.check(sorted(g2_rows) == sorted(
        set(row["path"] for row in g1["protected_files"]) |
        set(row["path"] for row in g2_protection["added_vs_g1_accepted"])),
        "G2 reconstructed path set is inconsistent")
    checker.check(canonical_protected_tree(g2_rows.values()) ==
                  EXPECTED_G2_TREE, "G2 tree reconstruction failed")
    return g1, acceptance, g2_manifest, g2_protection, g2_rows, checked


def verify_protection(checker, manifest, g1, g2_protection, g2_rows):
    protection_path = RUN / "analysis" / "protection_diff_vs_g2_accepted.json"
    checker.check(protection_path.is_file(), "protection diff is missing")
    protection = load(protection_path)
    checker.check(manifest["protected"]["comparison_sha256"] ==
                  sha256(protection_path),
                  "manifest protection comparison hash changed")
    checker.check(manifest["protected"]["g2_tree_sha256"] ==
                  EXPECTED_G2_TREE, "manifest G2 tree binding changed")
    checker.check(manifest["protected"]["g1_protected_after_sha256"] ==
                  EXPECTED_G1_AFTER_SHA,
                  "manifest G1 snapshot binding changed")
    checker.check(manifest["protected"]["g1_tree_sha256"] ==
                  EXPECTED_G1_TREE, "manifest G1 tree binding changed")
    checker.check(protection["g1_evidence"]["protected_tree_sha256"] ==
                  EXPECTED_G1_TREE, "protection G1 tree changed")
    checker.check(protection["g2_evidence"]["protected_tree_sha256"] ==
                  EXPECTED_G2_TREE, "protection G2 tree changed")
    checker.check(protection["g2_evidence"][
        "reconstruction_from_g1_plus_g2_changes"] is True,
        "G2 reconstruction flag was lost")
    checker.check(protection["authorization"][
        "only_authorized_changes"] is True,
        "G3 protection diff is not exactly authorized")
    checker.check(protection["removed_vs_g2_accepted"] == [],
                  "G3 protected removals exist")

    rows = current_entries(g1["protected_paths"])
    current_tree = canonical_protected_tree(rows)
    checker.check(current_tree == protection["current"][
        "full_protected_tree_sha256"],
        "current protected tree does not match the protection diff")
    checker.check(current_tree == manifest["protected"][
        "current_full_protected_tree_sha256"],
        "current protected tree does not match the manifest")
    checker.check(protection["current"]["unchanged_common_tree_matches"] is
                  True, "common protected tree changed")
    checker.check(protection["current"][
        "unchanged_common_tree_sha256_g2_reconstructed"] ==
        protection["current"]["unchanged_common_tree_sha256_current"],
        "unchanged common tree hashes differ")

    current = {row["path"]: row for row in rows}
    old = g2_rows
    added = sorted(set(current) - set(old))
    removed = sorted(set(old) - set(current))
    changed = sorted(
        path for path in set(old) & set(current)
        if (old[path]["type"], old[path]["sha256"], old[path]["size_bytes"])
        != (current[path]["type"], current[path]["sha256"],
            current[path]["size_bytes"]))
    checker.check(added == ALLOWED_NEW,
                  "protected additions are not the G3 whitelist")
    checker.check(changed == ALLOWED_CHANGED,
                  "protected changes are not the G3 whitelist")
    checker.check(removed == [], "protected removals are present")
    checker.check(manifest["protected"]["added_vs_g2_accepted"] == added,
                  "manifest additions do not match the current tree")
    checker.check(manifest["protected"]["changed_vs_g2_accepted"] == changed,
                  "manifest changes do not match the current tree")
    checker.check(manifest["protected"]["removed_vs_g2_accepted"] == [],
                  "manifest records protected removals")
    for relative in ALLOWED_NEW:
        source = manifest["source_files"].get(relative)
        checker.check(source is not None and source["sha256"] ==
                      current[relative]["sha256"],
                      "new protected file is not bound as a G3 source: " +
                      relative)
    for relative in ALLOWED_CHANGED:
        checker.check(manifest["source_files"][relative]["sha256"] ==
                      current[relative]["sha256"],
                      "changed protected file hash mismatch: " + relative)
    return protection, current_tree


def verify_model_case(checker, results_case, benchmark_case, index):
    name = EXPECTED_CASES[index]
    checker.check(results_case["name"] == name, "case order changed")
    checker.check(results_case["model_sha256"] ==
                  benchmark_case["model_sha256"],
                  name + " model digest differs from benchmark")
    checker.check(results_case["request_sha256"] ==
                  benchmark_case["request_sha256"],
                  name + " request digest differs from benchmark")
    serialized = benchmark_case["model_json"]
    model = json.loads(serialized)
    checker.check(canonical_bytes(model).decode("utf-8") == serialized,
                  name + " model JSON is not canonical")
    checker.check(hashlib.sha256(serialized.encode("utf-8")).hexdigest() ==
                  benchmark_case["model_sha256"],
                  name + " model JSON SHA-256 does not match")
    checker.check(set(model) == MODEL_KEYS,
                  name + " model top-level fields changed")
    checker.check(model["planning_context"] is None,
                  name + " unexpectedly embeds a planning context")
    for path, key, _ in recursive_keys(model):
        checker.check(key not in FORBIDDEN_MODEL_KEYS,
                      name + " model embeds forbidden field " + path)
    request_inputs = model["request_inputs"]
    request_sha = canonical_sha(request_inputs)
    checker.check(model["request_sha256"] == request_sha,
                  name + " model request_inputs digest changed")
    checker.check(request_sha == results_case["request_sha256"],
                  name + " result request digest changed")
    checker.check(model["timing"]["law"] ==
                  "virtual_rigid_a.timing.quintic_smoothstep.v1",
                  name + " timing law changed")
    checker.check(set(model["timing"]) == TIMING_KEYS,
                  name + " timing summary fields changed")
    checker.check(model["timing"]["S"] == results_case["S_m"],
                  name + " model S does not match the result")
    checker.check(model["timing"]["T_s"] == results_case["T_s"],
                  name + " model T does not match the result")
    checker.check(results_case["model_json_location"] ==
                  "benchmark/results.json:{} .model_json".format(name),
                  name + " model JSON location changed")

    timing = benchmark_case["timing"]
    checker.check(timing["T_s"] == results_case["T_s"],
                  name + " timing result T differs")
    checker.check(timing["duration_s"] == results_case["T_s"],
                  name + " timing result duration differs")
    checker.check(timing["total_S_m"] == results_case["S_m"],
                  name + " timing result S differs")

    roundtrip = results_case["roundtrip"]
    checker.check(roundtrip["equal_digest"] is True,
                  name + " roundtrip digest is not equal")
    checker.check(roundtrip["equal_json"] is True,
                  name + " roundtrip JSON is not equal")
    if name == "NO_MOTION_HOLD":
        hold = results_case["audit"]["hold"]
        checker.check(hold["hold_model"] == "explicit_zero_duration",
                      name + " hold model changed")
        checker.check(hold["inverse_division_free"] is True,
                      name + " hold inverse is not division-free")
        checker.check(hold["sample_times"] == [0.0],
                      name + " hold sample times changed")
        checker.check(hold["max_state_error"] == 0.0,
                      name + " hold state error changed")
        checker.check(results_case["sample"]["times"] == [0.0],
                      name + " hold sample is not exactly t=0")
        checker.check(results_case["sample"]["invalid"] and all(
            item["rejected"] is True for item in results_case["sample"][
                "invalid"]), name + " hold invalid inverse not rejected")
        checker.check(results_case["audit"]["whole_domain_certificate"] is
                      False, name + " hold claims a motion certificate")
        checker.check("certificate_note" in results_case["audit"],
                      name + " hold certificate note is missing")
        checker.check(results_case["timing_status"] == "NO_MOTION",
                      name + " hold status changed")
        checker.check(benchmark_case["dynamic_audit"]["metrics"]["kind"] ==
                      "hold", name + " hold audit kind changed")
        checker.check("differential" not in results_case["audit"] and
                      "time_inverse" not in results_case["audit"] and
                      "nodes" not in results_case["audit"],
                      name + " hold audit unexpectedly has motion metrics")
        selection = model["timing"]["selection"]
        checker.check(set(selection) == SELECTION_KEYS,
                      name + " hold selection fields changed")
        checker.check(selection["T0_s"] == 0.0 and
                      selection["interval_subdivisions_per_segment"] == 0 and
                      selection["metrics_at_T0_s"] == {} and
                      selection["required_duration_s"] == 0.0 and
                      selection["selected_duration_s"] == 0.0 and
                      selection["safety_factor"] == 1.0 and
                      selection["rescalings"] == 0,
                      name + " hold selection is not exact zero")
        checker.check(results_case["selection"] is None,
                      name + " hold result selection is not null")
        checker.check(results_case["sample"]["authoritative"] is False,
                      name + " hold sample claims authority")
        return

    checker.check(timing["status"] == "DYNAMIC_FEASIBLE",
                  name + " timing status is not DYNAMIC_FEASIBLE")
    checker.check(results_case["timing_status"] == "DYNAMIC_FEASIBLE",
                  name + " results timing status changed")
    checker.check(benchmark_case["dynamic_audit"]["passed"] is True,
                  name + " dynamic audit did not pass")
    checker.check(benchmark_case["solve"]["status"] == "GEOMETRY_FEASIBLE",
                  name + " geometry solve status changed")
    geometry = timing["geometry_audit"]
    checker.check(geometry["passed"] is True,
                  name + " geometry audit did not pass")
    checker.check(geometry["reason"] is None,
                  name + " geometry audit reason changed")
    sample_audit = geometry["metrics"]["sample_audit"]
    checker.check(check_float(
        checker, sample_audit["max_pairwise_rigidity_error_m"],
        name + " rigidity") <= 1e-6,
        name + " rigidity exceeds 1e-6 m")
    checker.check(check_float(
        checker, sample_audit["max_position_error_m"],
        name + " geometry position") <=
        sample_audit["position_tolerance_m"],
        name + " geometry position exceeds tolerance")
    c2 = geometry["c2"]
    checker.check(check_float(checker, c2["max_value_error"],
                              name + " c2 value") <=
                  c2["value_tolerance"],
                  name + " C2 value error exceeds tolerance")
    checker.check(check_float(checker, c2["max_slope_error"],
                              name + " c2 slope") <=
                  c2["slope_tolerance"],
                  name + " C2 slope error exceeds tolerance")
    checker.check(check_float(checker, c2["max_second_error"],
                              name + " c2 second") <=
                  c2["second_tolerance"],
                  name + " C2 second error exceeds tolerance")
    checker.check(check_float(checker, c2["max_abs_g"],
                              name + " c2 g") >= 0.0,
                  name + " C2 g magnitude is invalid")
    selection = model["timing"]["selection"]
    checker.check(set(selection) == SELECTION_KEYS,
                  name + " selection fields changed")
    checker.check(selection["T0_s"] == 1.0,
                  name + " selection T0 changed")
    checker.check(selection["interval_subdivisions_per_segment"] == 8,
                  name + " selection subdivisions changed")
    checker.check(selection["rescalings"] == 0,
                  name + " unexpected rescaling occurred")
    checker.check(selection["safety_factor"] == 1.01,
                  name + " safety factor changed")
    limits = request_inputs["limits"]
    metrics = selection["metrics_at_T0_s"]
    checker.check(set(metrics) == set(SCALED_METRICS),
                  name + " selection metric table changed")
    required_rows = {}
    for metric_name, (limit_name, order) in SCALED_METRICS.items():
        record = metrics[metric_name]
        checker.check(record["order"] == order,
                      name + " metric order changed: " + metric_name)
        checker.check(record["limit"] == limits[limit_name],
                      name + " metric limit is not the request limit: " +
                      metric_name)
        upper = check_float(checker, record["upper_at_T0_s"],
                            name + " upper " + metric_name)
        interval = record["upper_bound_interval"]
        checker.check(isinstance(interval, list) and len(interval) == 2,
                      name + " bound interval is not a pair: " + metric_name)
        s0 = check_float(checker, interval[0],
                         name + " interval s0 " + metric_name)
        s1 = check_float(checker, interval[1],
                         name + " interval s1 " + metric_name)
        checker.check(0.0 <= s0 <= s1 <= results_case["S_m"],
                      name + " bound interval is outside the path: " +
                      metric_name)
        checker.check(record["segment_index"] in range(8),
                      name + " metric segment index changed: " + metric_name)
        checker.check(record["wheel_index"] in range(4),
                      name + " metric wheel index changed: " + metric_name)
        checker.check(record["vehicle_id"] in
                      {row[0] for row in request_inputs["model"]},
                      name + " metric vehicle is not in the request: " +
                      metric_name)
        candidate = (upper / record["limit"]) ** (1.0 / order)
        reported = check_float(checker, record["required_duration_s"],
                               name + " required " + metric_name)
        checker.check(abs(candidate - reported) <= 1e-9 * max(1.0, candidate),
                      name + " metric candidate is not recomputable: " +
                      metric_name)
        required_rows[metric_name] = (reported, record)
    required = max(value for value, _ in required_rows.values())
    checker.check(abs(selection["required_duration_s"] - required) <=
                  1e-9 * max(1.0, required),
                  name + " required duration is not the maximum candidate")
    checker.check(abs(selection["selected_duration_s"] -
                      required * selection["safety_factor"]) <=
                  1e-9 * max(1.0, required),
                  name + " selected duration is not required*safety")
    checker.check(abs(selection["selected_duration_s"] -
                      results_case["T_s"]) <=
                  1e-9 * max(1.0, results_case["T_s"]),
                  name + " selected duration is not the model T")
    worst = selection["worst"]
    worst_name = max(required_rows, key=lambda key: required_rows[key][0])
    worst_value, worst_record = required_rows[worst_name]
    checker.check(abs(worst_value - required) <= 1e-9 * max(1.0, required),
                  name + " worst metric candidate is not binding")
    checker.check(worst["quantity"] == worst_name,
                  name + " worst quantity changed")
    checker.check(worst["order"] == worst_record["order"],
                  name + " worst order changed")
    checker.check(worst["segment_index"] ==
                  worst_record["segment_index"],
                  name + " worst segment changed")
    checker.check(worst["vehicle_id"] == worst_record["vehicle_id"],
                  name + " worst vehicle changed")
    checker.check(worst["wheel_index"] == worst_record["wheel_index"],
                  name + " worst wheel changed")
    checker.check(worst["s0"] == worst_record["upper_bound_interval"][0] and
                  worst["s1"] == worst_record["upper_bound_interval"][1],
                  name + " worst interval changed")
    checker.check(selection["wheel_angle_limit_rad"] ==
                  limits["steer_angle_max"],
                  name + " wheel angle limit is not the request limit")
    wheel_bound = check_float(checker, selection["wheel_angle_bound_rad"],
                              name + " wheel angle bound")
    checker.check(0.0 <= wheel_bound <= selection["wheel_angle_limit_rad"],
                  name + " wheel angle bound exceeds its limit")
    checker.check(results_case["roundtrip"]["equal_state_midpoint"] is True,
                  name + " midpoint roundtrip state changed")
    sample = results_case["sample"]
    checker.check(sample["authoritative"] is False,
                  name + " sample claims authority")
    checker.check(sample["states_equal_pointwise"] is True,
                  name + " sample states are not pointwise equal")
    checker.check(len(sample["times"]) == 3 and
                  sample["times"][0] == 0.0 and
                  abs(sample["times"][-1] - results_case["T_s"]) <= 1e-9,
                  name + " sample times changed")

    audit = benchmark_case["dynamic_audit"]
    checker.check(audit["reason"] is None, name + " audit reason changed")
    checker.check(audit["scope"] == "DYNAMICS",
                  name + " audit scope changed")
    checker.check(audit["check_count"] == EXPECTED_MOTION_AUDIT_CHECKS,
                  name + " audit check count changed")
    metrics_audit = audit["metrics"]
    checker.check(metrics_audit["kind"] == "motion",
                  name + " motion audit kind changed")
    checker.check(metrics_audit["whole_domain_certificate"] is True,
                  name + " whole-domain certificate missing")
    checker.check(metrics_audit["leaves"] == EXPECTED_CERTIFICATE_LEAVES,
                  name + " certificate leaves changed")
    pointwise = metrics_audit["pointwise_interval_checks"]
    checker.check(pointwise["leaf_count"] == EXPECTED_CERTIFICATE_LEAVES,
                  name + " pointwise leaf count changed")
    checker.check(pointwise["vehicle_count"] ==
                  EXPECTED_POINTWISE_VEHICLE_CHECKS,
                  name + " pointwise vehicle count changed")
    for ratio_name in ("max_jerk_ratio", "max_component_ratio",
                       "max_p_sss_ratio"):
        ratio_value = check_float(checker, pointwise[ratio_name],
                                  name + " pointwise " + ratio_name)
        checker.check(ratio_value <= 1.0,
                      name + " pointwise ratio exceeds one: " + ratio_name)
    worst = pointwise["worst"]
    checker.check(worst is not None and
                  check_float(checker, worst["ratio"],
                              name + " pointwise worst") <= 1.0,
                  name + " pointwise worst exceeds one")
    checker.check(worst["quantity"] in
                  {"jerk", "jerk_x", "jerk_y", "p_sss_x", "p_sss_y"},
                  name + " pointwise worst quantity changed")
    checker.check(finite(worst["actual"]) and finite(worst["bound"]),
                  name + " pointwise worst value is not finite")
    checker.check(set(metrics_audit["max_utilization"]) == UTILIZATION_KEYS,
                  name + " utilization metric set changed")
    for metric_name, value in metrics_audit["max_utilization"].items():
        number = check_float(checker, value,
                             name + " utilization " + metric_name)
        checker.check(number <= 1.0 + 1e-9,
                      name + " utilization exceeds one: " + metric_name)
    nodes = metrics_audit["nodes"]
    checker.check(nodes["internal_node_count"] == 7,
                  name + " internal node count changed")
    checker.check(len(nodes["records"]) == 7,
                  name + " node record count changed")
    checker.check(check_float(checker, nodes["max_continuity_error"],
                              name + " node continuity") <= 1e-9,
                  name + " node continuity error exceeds tolerance")
    checker.check(check_float(checker, nodes["max_state_error"],
                              name + " node state") <= 1e-9,
                  name + " node state error exceeds tolerance")
    checker.check(check_float(checker, nodes["max_inverse_error_s"],
                              name + " node inverse") <= 1e-9,
                  name + " node inverse error exceeds tolerance")
    checker.check(check_float(checker, nodes["max_jump_magnitude"],
                              name + " node jump") >= 0.0,
                  name + " node jump magnitude is negative")
    previous_s = -1.0
    previous_t = -1.0
    for node_index, record in enumerate(nodes["records"]):
        checker.check(record["auto_region"] == "motion",
                      name + " node auto region changed")
        checker.check(record["left_region"] == "motion" and
                      record["right_region"] == "motion",
                      name + " internal node left/right region changed")
        s_value = check_float(checker, record["s"],
                              name + " node s {}".format(node_index))
        t_value = check_float(checker, record["t"],
                              name + " node t {}".format(node_index))
        checker.check(s_value > previous_s,
                      name + " node s is not increasing")
        checker.check(t_value > previous_t,
                      name + " node t is not increasing")
        checker.check(0.0 < s_value < results_case["S_m"] and
                      0.0 < t_value < results_case["T_s"],
                      name + " internal node outside the motion domain")
        previous_s, previous_t = s_value, t_value
        check_float(checker, record["w"], name + " node w")
        checker.check(len(record["left_jerk"]) == 3 and
                      len(record["right_jerk"]) == 3,
                      name + " node jerk vector shape changed")
        for side in ("left_jerk", "right_jerk"):
            for vehicle in record[side]:
                checker.check(len(vehicle) == 2,
                              name + " node " + side + " width changed")
                for value in vehicle:
                    checker.check(finite(value),
                                  name + " node " + side + " is not finite")
        checker.check(check_float(checker, record["max_continuity_error"],
                                  name + " node continuity record") <= 1e-9,
                      name + " node continuity record exceeds tolerance")
        checker.check(check_float(checker, record["inverse_error_s"],
                                  name + " node inverse record") <= 1e-9,
                      name + " node inverse record exceeds tolerance")
        for side in ("left", "right"):
            node_pointwise = record["pointwise_interval"][side]
            for ratio_name in ("jerk_ratio", "component_ratio",
                               "p_sss_ratio"):
                checker.check(check_float(
                    checker, node_pointwise[ratio_name],
                    name + " node " + side + " " + ratio_name) <= 1.0,
                    name + " node " + side +
                    " pointwise ratio exceeds one")
            node_worst = node_pointwise["worst"]
            checker.check(node_worst is not None and
                          node_worst["label"].startswith(
                              "node.{}[".format(node_index)),
                          name + " node " + side +
                          " pointwise label changed")
            checker.check(check_float(
                checker, node_worst["ratio"],
                name + " node " + side + " worst") <= 1.0,
                name + " node " + side + " pointwise worst exceeds one")
            checker.check(finite(node_worst["actual"]) and
                          finite(node_worst["bound"]),
                          name + " node " + side +
                          " pointwise value is not finite")

    bounds = benchmark_case["dynamic_audit"]["whole_segment_bounds"]
    checker.check(len(bounds) == 8, name + " segment bound count changed")
    checker.check(sum(row["leaves"] for row in bounds) == 64,
                  name + " segment leaf count changed")
    for segment_index, row in enumerate(bounds):
        checker.check(row["certified"] is True,
                      name + " segment bound is not certified")
        checker.check(row["max_depth"] <= 20,
                      name + " bound max depth exceeds 20")
        checker.check(check_float(checker, row["h"],
                                  name + " segment h") > 0.0,
                      name + " segment h is not positive")
        checker.check(set(row["metrics"]) == set(SCALED_METRICS),
                      name + " segment metrics changed")
        for metric_name, record in row["metrics"].items():
            checker.check(record["limit"] > 0.0,
                          name + " segment limit is not positive")
            checker.check(0.0 <= record["q0"] <= record["q1"] <= 1.0,
                          name + " segment q interval changed")
            checker.check(0.0 <= record["tau0"] <= record["tau1"] <= 1.0,
                          name + " segment tau interval changed")
            upper = check_float(checker, record["upper_bound"],
                                name + " segment bound " + metric_name)
            checker.check(upper <= record["limit"] + 1e-9,
                          name + " whole-domain bound exceeds limit: " +
                          metric_name)

    differential = metrics_audit["differential"]
    checker.check(differential["checked_sample_count"] == 80,
                  name + " differential sample count changed")
    checker.check(differential["skipped_sample_count"] == 0,
                  name + " differential stencils were skipped")
    checker.check(differential["tolerances"] == DIFFERENTIAL_TOLERANCES,
                  name + " differential tolerances changed")
    checker.check(set(differential["max_errors"]) ==
                  set(DIFFERENTIAL_TOLERANCES),
                  name + " differential metric set changed")
    for metric_name, tolerance in DIFFERENTIAL_TOLERANCES.items():
        error = check_float(checker,
                            differential["max_errors"][metric_name],
                            name + " differential " + metric_name)
        checker.check(error <= tolerance,
                      name + " differential error exceeds tolerance: " +
                      metric_name)
    inverse = metrics_audit["time_inverse"]
    checker.check(inverse["sample_count"] == 100,
                  name + " inverse sample count changed")
    checker.check(len(inverse["samples"]) == 100,
                  name + " inverse sample records changed")
    checker.check(inverse["tolerance_s"] == 1e-9,
                  name + " inverse tolerance changed")
    checker.check(check_float(checker, inverse["max_error_s"],
                              name + " inverse maximum") <= 1e-9,
                  name + " inverse maximum exceeds tolerance")
    for sample_index, sample in enumerate(inverse["samples"]):
        checker.check(finite(sample["s"]) and
                      0.0 <= sample["s"] <= results_case["S_m"],
                      name + " inverse sample s is invalid")
        checker.check(finite(sample["time_at_s"]),
                      name + " inverse sample time is invalid")
        checker.check(check_float(checker, sample["error_s"],
                                  name + " inverse sample {}".format(
                                      sample_index)) <= 1e-9,
                      name + " inverse sample exceeds tolerance")

    endpoints = metrics_audit["endpoints"]
    checker.check(check_float(checker, endpoints["max_state_error"],
                              name + " endpoint state") <= 1e-9,
                  name + " endpoint state error exceeds tolerance")
    checker.check(endpoints["jerk_semantics"] ==
                  "one-sided endpoint jerk retained, not required zero",
                  name + " endpoint jerk semantics changed")
    checker.check(endpoints["initial_region"] == "initial_hold" and
                  endpoints["terminal_region"] == "terminal_hold",
                  name + " endpoint regions changed")

    short = results_case["short_duration"]
    checker.check(short["rejected"] is True,
                  name + " short-duration counterexample was not rejected")
    checker.check(short["reason"] == "DYNAMIC_LIMIT",
                  name + " short-duration rejection reason changed")
    failure = short["first_failure"]
    checker.check(failure["quantity"] in
                  {"speed", "acceleration", "jerk", "yaw_rate",
                   "yaw_acceleration", "yaw_jerk", "lateral_acceleration",
                   "wheel_speed", "steer_rate", "steer_acceleration"},
                  name + " short-duration failure metric changed")
    checker.check(failure["value"] > failure["limit"],
                  name + " short-duration failure does not exceed its limit")
    checker.check(failure["vehicle_id"] in
                  {row[0] for row in request_inputs["model"]},
                  name + " short-duration failure vehicle is invalid")
    checker.check(short["check_count"] > 0,
                  name + " short-duration audit performed no checks")
    tamper = results_case["tamper"]
    checker.check(tamper == {
        "timing_T": "MODEL_TIMING_INCONSISTENT",
        "extra_field": "UNSUPPORTED_MODEL_FIELD",
        "self_digest": "UNSUPPORTED_MODEL_FIELD",
    }, name + " tamper rejection record changed")
    checker.check(benchmark_case["tamper"] == tamper,
                  name + " benchmark tamper record changed")


def verify_results(checker):
    results = load(RUN / "results.json")
    checker.check(results["status"] == "PASS_PENDING_CODEX_RECHECK",
                  "results status is not pending Codex recheck")
    checker.check(results["codex_acceptance"] == "NOT_RUN_NOT_CLAIMED",
                  "results claim Codex acceptance")
    scope = results["scope"]
    checker.check(scope["timing_and_dynamics_proven"] is True,
                  "results do not record the G3 proof")
    for key in ("collision_checked", "executable", "map_used", "ros_used",
                "gazebo_used", "hardware_used", "time_optimal",
                "g4_entered"):
        checker.check(scope[key] is False,
                      "results scope claims " + key)
    for name in EXPECTED_TEST_COUNTS:
        verify_test_log(checker, results, name)
    benchmark = results["benchmark"]
    checker.check(benchmark["status"] == "PASS",
                  "results benchmark status changed")
    checker.check(benchmark["check_count"] == EXPECTED_CHECK_COUNT,
                  "results benchmark check count changed")
    checker.check(benchmark["failed"] == [],
                  "results benchmark failed checks are present")
    checker.check(benchmark["codex_acceptance"] ==
                  "NOT_RUN_NOT_CLAIMED",
                  "results benchmark claims Codex acceptance")
    summary = load(RUN / "benchmark" / "summary.json")
    checker.check(summary["status"] == "PASS",
                  "benchmark summary status changed")
    checks = summary["checks"]
    checker.check(len(checks) == EXPECTED_CHECK_COUNT,
                  "benchmark summary check count changed")
    checker.check(all(check["pass"] is True for check in checks),
                  "benchmark summary contains a failed check")
    checker.check(len({check["name"] for check in checks}) ==
                  EXPECTED_CHECK_COUNT,
                  "benchmark check names are not unique")
    checker.check(summary["codex_acceptance"] ==
                  "NOT_RUN_NOT_CLAIMED",
                  "benchmark summary claims Codex acceptance")
    checker.check(results["benchmark"]["check_count"] == len(checks),
                  "results/benchmark check counts differ")
    benchmark_results = load(RUN / "benchmark" / "results.json")
    checker.check(sorted(benchmark_results) == sorted(EXPECTED_CASES),
                  "benchmark result cases changed")
    checker.check([case["name"] for case in results["model_cases"]] ==
                  EXPECTED_CASES, "results model case order changed")
    for index, name in enumerate(EXPECTED_CASES):
        verify_model_case(checker, results["model_cases"][index],
                          benchmark_results[name], index)
    aggregate = results["aggregate"]
    checker.check(aggregate["benchmark_check_count"] ==
                  EXPECTED_CHECK_COUNT,
                  "aggregate benchmark count changed")
    checker.check(aggregate["dynamic_feasible_cases"] == 8,
                  "dynamic feasible case count changed")
    checker.check(aggregate["no_motion_cases"] == 1,
                  "NO_MOTION case count changed")
    checker.check(aggregate["short_duration_rejections"] == 8,
                  "short-duration rejection count changed")
    checker.check(aggregate["tamper_rejections"] == 8,
                  "tamper rejection count changed")
    checker.check(aggregate["inverse_samples_per_case"] == 100,
                  "inverse sample count aggregate changed")
    checker.check(aggregate["time_inverse_tolerance_s"] == 1e-9,
                  "inverse tolerance aggregate changed")
    checker.check(aggregate["differential_tolerances"] ==
                  DIFFERENTIAL_TOLERANCES,
                  "differential tolerance aggregate changed")
    checker.check(set(aggregate["max_differential_errors"]) ==
                  set(DIFFERENTIAL_TOLERANCES),
                  "differential maximum metric set changed")
    for metric_name, tolerance in DIFFERENTIAL_TOLERANCES.items():
        checker.check(check_float(checker, aggregate[
            "max_differential_errors"][metric_name],
            "aggregate differential " + metric_name) <= tolerance,
            "aggregate differential error exceeds tolerance: " +
            metric_name)
    checker.check(check_float(checker,
                              aggregate["max_time_inverse_error_s"],
                              "aggregate inverse") <= 1e-9,
                  "aggregate inverse error exceeds tolerance")
    checker.check(check_float(checker,
                              aggregate["max_node_continuity_error"],
                              "aggregate node continuity") <= 1e-9,
                  "aggregate node continuity exceeds tolerance")
    checker.check(check_float(checker,
                              aggregate["max_node_state_error"],
                              "aggregate node state") <= 1e-9,
                  "aggregate node state exceeds tolerance")
    checker.check(check_float(checker, aggregate[
        "max_actual_utilization"], "aggregate utilization") <= 1.0 + 1e-9,
        "aggregate utilization exceeds one")
    checker.check(aggregate["total_rescalings"] == 0,
                  "aggregate rescalings changed")
    checker.check(aggregate["all_dynamic_audits_passed"] is True,
                  "not all dynamic audits passed")
    checker.check(aggregate["pointwise_leaf_count"] ==
                  EXPECTED_CERTIFICATE_LEAVES * 8,
                  "aggregate pointwise leaf count changed")
    checker.check(aggregate["pointwise_vehicle_count"] ==
                  EXPECTED_POINTWISE_VEHICLE_CHECKS * 8,
                  "aggregate pointwise vehicle count changed")
    for ratio_name in ("max_pointwise_jerk_ratio",
                       "max_pointwise_component_ratio",
                       "max_pointwise_p_sss_ratio"):
        checker.check(check_float(checker, aggregate[ratio_name],
                                  "aggregate " + ratio_name) <= 1.0,
                      "aggregate pointwise ratio exceeds one: " +
                      ratio_name)
    checker.check(results["legacy"]["disclosed_wrapper"]["status"] ==
                  "FAIL", "legacy disclosure status changed")
    return results, benchmark_results


def contained_flags(value):
    if isinstance(value, dict):
        for key, child in value.items():
            if key == "contained":
                yield child
            for item in contained_flags(child):
                yield item
    elif isinstance(value, list):
        for child in value:
            for item in contained_flags(child):
                yield item


def verify_interval_coverage(checker, results):
    coverage = results["interval_coverage"]
    location = coverage["numerics_location"]
    checker.check(location == "analysis/g3_rework_numerics.json",
                  "numerics evidence location changed")
    numerics_path = RUN / location
    checker.check(numerics_path.is_file(),
                  "numerics evidence file is missing")
    if not numerics_path.is_file():
        return
    checker.check(sha256(numerics_path) == coverage["numerics_sha256"],
                  "numerics evidence SHA-256 changed")
    numerics = load(numerics_path)
    checker.check(numerics["schema"] == NUMERICS_SCHEMA,
                  "numerics schema changed")
    checker.check(numerics["generated_by"] ==
                  "analysis/g3_rework_numerics.py",
                  "numerics generator changed")

    degenerate = numerics["degenerate_counterexample"]
    checker.check(degenerate["expected_p_sss"] == [-0.04, 0.099],
                  "degenerate p_sss changed")
    checker.check(degenerate["expected_p_sss_norm"] ==
                  EXPECTED_DEGENERATE_NORM,
                  "degenerate expected norm changed")
    checker.check(degenerate["input"] == {
        "a_time": 0.0, "eta": 0.1, "g": 0.0, "j_time": 0.0,
        "k": 0.1, "offset": [1.0, 0.0], "u": 0.0, "w": 1.0},
        "degenerate Codex input changed")
    for implementation in ("timing", "audit"):
        record = degenerate[implementation]
        for key in ("covers_expected_components", "covers_expected_norm"):
            checker.check(all(record[key]) if isinstance(record[key], list)
                          else record[key] is True,
                          "degenerate " + implementation + " " + key)
        checker.check(check_float(
            checker, record["jerk"],
            "degenerate " + implementation + " jerk") >=
            EXPECTED_DEGENERATE_NORM,
            "degenerate " + implementation + " jerk is below the true norm")
        checker.check(check_float(
            checker, record["p_sss_norm_upper"],
            "degenerate " + implementation + " p_sss norm") >=
            EXPECTED_DEGENERATE_NORM,
            "degenerate " + implementation + " p_sss bound is too small")
        for key in ("jerk_component_norm_upper",):
            checker.check(check_float(
                checker, record[key], "degenerate " + implementation + key) >=
                EXPECTED_DEGENERATE_NORM,
                "degenerate component-norm bound is too small")
        for axis in (0, 1):
            z3 = record["p_sss_local_interval"][axis]
            jerk = record["jerk_local_interval"][axis]
            checker.check(z3[0] <= degenerate["expected_p_sss"][axis] <= z3[1],
                          "degenerate z3 interval does not contain value")
            checker.check(finite(z3[0]) and finite(z3[1]),
                          "degenerate z3 interval is not finite")
            checker.check(finite(jerk[0]) and finite(jerk[1]),
                          "degenerate jerk interval is not finite")
    old = degenerate["old_defective_bounds"]
    checker.check(old["timing_jerk"] < EXPECTED_DEGENERATE_NORM and
                  old["audit_jerk"] < EXPECTED_DEGENERATE_NORM,
                  "old defective bounds did not underestimate the true norm")
    checker.check(coverage["degenerate_counterexample"][
        "expected_p_sss_norm"] == EXPECTED_DEGENERATE_NORM,
        "results degenerate norm changed")
    checker.check(coverage["degenerate_counterexample"]["timing_jerk"] >=
                  EXPECTED_DEGENERATE_NORM and
                  coverage["degenerate_counterexample"]["audit_jerk"] >=
                  EXPECTED_DEGENERATE_NORM,
                  "results degenerate coverage changed")

    narrow = numerics["narrow_interval_coverage"]
    checker.check(narrow["case_count"] == 24,
                  "narrow interval case count changed")
    checker.check(narrow["failure_count"] == 0,
                  "narrow interval failures are present")
    checker.check(len(narrow["records"]) == 24,
                  "narrow interval record count changed")
    implementations = set()
    offsets = set()
    for record in narrow["records"]:
        implementations.add(record["implementation"])
        offsets.add(tuple(record["offset"]))
        checker.check(record["all_contained"] is True,
                      "narrow interval case is not contained")
        checks = record["checks"]
        checker.check(set(checks) == {
            "jerk_0", "jerk_1", "jerk_component_norm", "jerk_norm",
            "p_sss_0", "p_sss_1", "p_sss_norm"},
            "narrow interval check set changed")
        checker.check(all(c["contained"] is True for c in checks.values()),
                      "narrow interval check failed")
        for axis in (0, 1):
            for prefix in ("jerk_", "p_sss_"):
                check = checks[prefix + str(axis)]
                checker.check(check["interval"][0] <= check["actual"] <=
                              check["interval"][1],
                              "narrow signed interval does not contain value")
        checker.check(checks["jerk_norm"]["actual"] <=
                      checks["jerk_norm"]["upper"],
                      "narrow jerk norm exceeds its bound")
        checker.check(checks["jerk_component_norm"]["actual"] <=
                      checks["jerk_component_norm"]["upper"],
                      "narrow jerk component norm exceeds its bound")
        checker.check(checks["p_sss_norm"]["actual"] <=
                      checks["p_sss_norm"]["upper"],
                      "narrow p_sss norm exceeds its bound")
    checker.check(implementations == {"timing", "audit"},
                  "narrow intervals do not cover both implementations")
    checker.check(offsets == {(1.0, 0.0), (0.0, 1.0), (0.0, -1.0)},
                  "narrow intervals do not cover all three vehicles")
    checker.check(coverage["narrow_interval_case_count"] == 24 and
                  coverage["narrow_interval_failure_count"] == 0,
                  "results narrow intervals changed")

    paths = numerics["positive_paths"]
    checker.check([path["name"] for path in paths] == [
        "positive_curvature_cubic", "negative_curvature_cubic"],
        "positive path names changed")
    signs = set()
    for path in paths:
        checker.check(path["all_contained"] is True,
                      path["name"] + " is not fully contained")
        checker.check(path["sample_count"] == 20 and
                      len(path["samples"]) == 20,
                      path["name"] + " interior sample count changed")
        checker.check(path["node_sample_count"] == 3 and
                      len(path["node_samples"]) == 3,
                      path["name"] + " node sample count changed")
        front = path["front_vehicle"]
        checker.check(front["sample_count"] == 20 and
                      front["nonzero_k_times_eta"] > 0,
                      path["name"] + " front vehicle lacks nonzero k*eta")
        for pair in front["sign_combinations"]:
            signs.add((int(pair[0]), int(pair[1])))
        for sample in path["samples"]:
            checker.check(sample["k_times_eta"] != 0.0,
                          path["name"] + " sample has zero k*eta")
            checker.check(sample["comparison"]["contained"] is True,
                          path["name"] + " sample comparison is not contained")
            checker.check(all(contained_flags(sample["comparison"])),
                          path["name"] + " sample containment flag failed")
            checker.check(len(sample["comparison"]["vehicles"]) == 3,
                          path["name"] + " sample vehicle count changed")
            checker.check(set(sample["comparison"]["max_ratios"]) == {
                "jerk_component", "jerk_norm", "p_sss_component",
                "p_sss_norm"}, path["name"] + " sample ratio set changed")
            for ratio_name in ("jerk_component", "jerk_norm",
                               "p_sss_component", "p_sss_norm"):
                check_float(
                    checker, sample["comparison"]["max_ratios"][ratio_name],
                    path["name"] + " " + ratio_name)
            for ratio_name in ("jerk_norm", "p_sss_norm"):
                checker.check(check_float(
                    checker, sample["comparison"]["max_ratios"][ratio_name],
                    path["name"] + " " + ratio_name) <= 1.0,
                    path["name"] + " sample norm ratio exceeds one")
        for node in path["node_samples"]:
            for side in ("left_comparison", "right_comparison"):
                checker.check(node[side]["contained"] is True,
                              path["name"] + " node " + side +
                              " is not contained")
                checker.check(all(contained_flags(node[side])),
                              path["name"] + " node " + side +
                              " containment flag failed")
                checker.check(len(node[side]["vehicles"]) == 3,
                              path["name"] + " node vehicle count changed")
                for ratio_name in ("jerk_component", "jerk_norm",
                                   "p_sss_component", "p_sss_norm"):
                    check_float(
                        checker, node[side]["max_ratios"][ratio_name],
                        path["name"] + " node " + ratio_name)
                for ratio_name in ("jerk_norm", "p_sss_norm"):
                    checker.check(check_float(
                        checker, node[side]["max_ratios"][ratio_name],
                        path["name"] + " node " + ratio_name) <= 1.0,
                        path["name"] + " node norm ratio exceeds one")
    checker.check(signs == {(-1, -1), (-1, 1), (1, -1), (1, 1)},
                  "positive paths do not cover all four sign pairs")
    checker.check([path["audit"]["passed"] for path in paths] == [True, True],
                  "a positive path audit did not pass")
    checker.check([path["audit"]["leaves"] for path in paths] == [32, 32],
                  "positive path certificate leaves changed")
    for path in paths:
        checker.check(path["audit"]["pointwise_interval_checks"][
            "leaf_count"] == 32,
            path["name"] + " pointwise leaf count changed")
        checker.check(path["audit"]["pointwise_interval_checks"][
            "vehicle_count"] == 96,
            path["name"] + " pointwise vehicle count changed")
    checker.check(len(coverage["positive_paths"]) == 2,
                  "results positive path count changed")
    result_signs = set()
    for path in coverage["positive_paths"]:
        checker.check(path["all_contained"] is True,
                      "results positive path containment changed")
        for pair in path["front_sign_pairs"]:
            result_signs.add((int(pair[0]), int(pair[1])))
    checker.check(result_signs == signs,
                  "results positive sign pairs changed")

    negative = numerics["jerk_overlimit_negative"]
    checker.check(negative["limits"]["jerk_max"] == 0.2,
                  "negative jerk limit changed")
    checker.check(negative["accepted_plan"]["audit_passed"] is True and
                  negative["accepted_plan"]["status"] == "DYNAMIC_FEASIBLE",
                  "accepted negative-fixture plan is not feasible")
    shortened = negative["shortened_plan"]
    checker.check(shortened["audit_passed"] is False and
                  shortened["audit_reason"] == "DYNAMIC_LIMIT",
                  "shortened plan was not rejected for a dynamic limit")
    checker.check(shortened["first_failure"]["quantity"] == "jerk",
                  "shortened plan failure is not jerk")
    checker.check(shortened["first_failure"]["value"] >
                  negative["limits"]["jerk_max"],
                  "shortened plan first failure does not exceed the limit")
    away = shortened["scanned_maximum_away_from_endpoints"]
    checker.check(away["jerk"] > negative["limits"]["jerk_max"],
                  "dense scan has no away-from-endpoint over-limit witness")
    checker.check(0.01 <= away["tau"] <= 0.99,
                  "dense-scan witness is not away from the endpoints")
    checker.check(0.0 <= away["s"] <= 3.0,
                  "dense-scan witness s is outside the path")
    checker.check(shortened["scan_over_limit_count_away_from_endpoints"] > 0,
                  "dense scan over-limit count changed")
    result_negative = coverage["jerk_overlimit_negative"]
    checker.check(result_negative["audit_reason"] == "DYNAMIC_LIMIT",
                  "results negative audit reason changed")
    checker.check(result_negative["first_failure"] ==
                  shortened["first_failure"],
                  "results negative first failure changed")
    checker.check(result_negative["interior_overlimit_samples"] ==
                  shortened["scan_over_limit_count_away_from_endpoints"],
                  "results negative interior witness count changed")
    checker.check(result_negative["interior_scan_maximum"]["jerk"] ==
                  away["jerk"],
                  "results negative interior maximum changed")

    closed = numerics["broad_interval_fail_closed"]
    checker.check(closed["coarse"]["passed"] is False and
                  closed["coarse"]["reason"] == "BOUND_UNCONFIRMED",
                  "coarse interval did not fail closed")
    checker.check(closed["refined"]["passed"] is True and
                  closed["refined"]["leaves"] > 3,
                  "refined interval did not confirm the bound")
    result_closed = coverage["broad_interval_fail_closed"]
    checker.check(result_closed["coarse"]["reason"] == "BOUND_UNCONFIRMED" and
                  result_closed["refined"]["passed"] is True,
                  "results fail-closed evidence changed")
    checker.check(result_closed["coarse"]["first_failure"] ==
                  closed["coarse"]["first_failure"],
                  "results coarse first failure changed")
    checker.check(result_closed["refined"]["leaves"] ==
                  closed["refined"]["leaves"],
                  "results refined leaf count changed")


def verify_legacy(checker, results):
    legacy = load(RUN / "legacy" / "regression_results.json")
    checker.check(legacy["status"] == "FAIL",
                  "legacy regression status is not truthfully FAIL")
    statuses = {row["case_id"]: row["status"] for row in legacy["records"]}
    checker.check(statuses == {
        "timing_continuous": "FAIL",
        "timing_formation": "PASS",
        "fr02_a_timing": "PASS",
        "fr02_a_trajectory": "PASS",
        "fr02_b_quality": "PASS",
    }, "legacy case statuses changed")
    for row in legacy["records"]:
        if row["case_id"] == "timing_continuous":
            checker.check(row["returncode"] == 1,
                          "legacy failure returncode changed")
        else:
            checker.check(row["returncode"] == 0,
                          "legacy pass returncode changed")
    disclosed = results["legacy"]["disclosed_wrapper"]["cases"]
    checker.check({name: record["status"] for name, record in
                   disclosed.items()} == statuses,
                  "results legacy cases differ from the regression record")
    gate = (RUN / "logs" / "legacy_unmodified_gate.log").read_text(
        encoding="utf-8")
    checker.check("UNKNOWN_WORKSPACE_MODIFICATIONS" in gate,
                  "unmodified-gate failure text missing")
    checker.check(results["legacy"]["unmodified_g0_gate"]["status"] ==
                  "BLOCKED", "gate result is not BLOCKED")
    checker.check(results["legacy"]["unmodified_g0_gate"]["exit_code"] ==
                  1, "gate exit code changed")
    probe = (RUN / "logs" / "legacy_baseline_probe.log").read_text(
        encoding="utf-8")
    checker.check("KeyError: 'dynamic_peaks'" in probe,
                  "pre-existing probe failure text missing")
    checker.check("probe_exit=1" in probe,
                  "pre-existing probe exit is missing")
    proof = results["legacy"]["pre_existing_failure_proof"]
    checker.check(proof["failed_test_file_sha256"] ==
                  "b7a36fd70c5d2bc5535334836d958a10afa10109f0c7cd2b226745179c7eee0c",
                  "probe test hash changed")
    checker.check(proof["failed_source_file_sha256"] ==
                  "2d1e9b182028e63d5fa9b25566ea0b9ab0cccb4ea7d3921ccfe651f3186ea8ef",
                  "probe source hash changed")
    checker.check(proof["exit_code"] == 1,
                  "probe exit code changed")
    checker.check(proof["log"] == "logs/legacy_baseline_probe.log",
                  "probe log path changed")


def verify_git_and_environment(checker, manifest):
    environment = load(RUN / "environment.json")
    checker.check(environment["run_id"] == RUN.name,
                  "environment run id changed")
    checker.check(environment["git"]["head"] == BASELINE_HEAD,
                  "environment HEAD changed")
    checker.check(environment["solver"]["ipopt_plugin_available"] is True,
                  "IPOPT availability changed")
    checker.check(environment["solver"]["casadi_version"] == "3.7.2",
                  "CasADi version changed")
    checker.check(environment["host"]["python_version"] == "3.8.10",
                  "Python version changed")
    checker.check(environment["process_environment"][
        "PYTHONDONTWRITEBYTECODE"] == "1",
        "bytecode environment was not disabled")

    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    checker.check(head == BASELINE_HEAD, "git HEAD changed")
    checker.check(branch == "feature/virtual-rigid-formation-a",
                  "git branch changed")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD"],
        cwd=str(REPO_ROOT), check=False).returncode
    checker.check(ancestor == 0, "baseline is not an ancestor")
    status = git("status", "--short").splitlines()
    checker.check(status == manifest["git"]["status_short"],
                  "git status changed during evidence finalization")
    staged = git("diff", "--cached", "--binary")
    checker.check(staged == "", "staged changes are present")
    checker.check(hashlib.sha256(staged.encode("utf-8")).hexdigest() ==
                  manifest["git"]["staged_diff_sha256"],
                  "staged diff hash changed")
    checker.check(manifest["git"]["committed"] == "NO" and
                  manifest["git"]["pushed"] == "NO" and
                  manifest["git"]["branch_created"] == "NO",
                  "manifest claims a git operation")
    checker.check(manifest["git"]["baseline_is_ancestor"] is True,
                  "manifest baseline ancestry changed")


def main():
    checker = Checker()
    checker.check(MANIFEST.is_file(), "source manifest is missing")
    if not MANIFEST.is_file():
        raise SystemExit("VERIFY_FAILED: source manifest is missing")
    manifest = load(MANIFEST)
    checker.check(manifest["schema"] == SCHEMA,
                  "manifest schema changed")
    checker.check(manifest["stage"] == "G3", "manifest stage changed")
    checker.check(manifest["run_id"] == RUN.name,
                  "manifest run id changed")
    checker.check(manifest["baseline_head"] == BASELINE_HEAD,
                  "manifest baseline changed")
    checker.check(manifest["excluded_self_referential"] ==
                  EXPECTED_EXCLUSIONS,
                  "manifest self-exclusion list changed")
    g1, acceptance, g2_manifest, g2_protection, g2_rows, g2_checked = \
        verify_g2(checker)
    verify_protection(checker, manifest, g1, g2_protection, g2_rows)

    g2_sources = g2_manifest["source_files"]
    expected_sources = sorted(set(g2_sources) | set(G3_SOURCES))
    checker.check(sorted(manifest["source_files"]) == expected_sources,
                  "manifest source set is not the 31 accepted+G3 files")
    checker.check(manifest["source_file_count"] == EXPECTED_SOURCE_COUNT,
                  "manifest source count is not 31")
    checker.check(manifest["g3_source_files"] == G3_SOURCES,
                  "manifest G3 source list changed")
    for relative in expected_sources:
        record = manifest["source_files"][relative]
        path = REPO_ROOT / relative
        checker.check(path.is_file() and sha256(path) == record["sha256"] and
                      path.stat().st_size == record["bytes"],
                      "source hash changed: " + relative)
    for relative in G3_SOURCES:
        checker.check(relative not in g2_sources,
                      "a G3 source was already in the accepted G2 manifest")
    changed = sorted(
        relative for relative in g2_sources
        if manifest["source_files"][relative]["sha256"] !=
        g2_sources[relative]["sha256"])
    checker.check(changed == ALLOWED_CHANGED,
                  "changed accepted source set is not the G3 whitelist")
    checker.check(manifest["changed_accepted_source_list"] == changed,
                  "manifest changed source list changed")
    for relative, record in manifest["changed_accepted_sources"].items():
        checker.check(relative in g2_sources,
                      "changed-source record is not an accepted source")
        checker.check(record["g2_sha256"] ==
                      g2_sources[relative]["sha256"],
                      "changed-source G2 hash changed")
        checker.check(record["g3_sha256"] ==
                      manifest["source_files"][relative]["sha256"],
                      "changed-source G3 hash changed")
        checker.check(record["changed"] == (record["g2_sha256"] !=
                                            record["g3_sha256"]),
                      "changed-source flag is inconsistent")
    checker.check(set(manifest["reference_documents"]) ==
                  set(REFERENCE_DOCUMENTS),
                  "reference document set changed")
    for relative, record in manifest["reference_documents"].items():
        checker.check(file_record(REPO_ROOT / relative) == record,
                      "reference document hash changed: " + relative)

    actual_run_files = {}
    for path in sorted(RUN.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(RUN).as_posix()
        if relative in EXPECTED_EXCLUSIONS:
            continue
        actual_run_files[relative] = file_record(path)
    checker.check(set(actual_run_files) == set(manifest["run_files"]),
                  "run file set changed")
    for relative, record in actual_run_files.items():
        checker.check(manifest["run_files"].get(relative) == record,
                      "run file hash changed: " + relative)
    checker.check(manifest["run_file_count"] == len(actual_run_files),
                  "run file count changed")
    checker.check(manifest["run_total_bytes"] == sum(
        record["bytes"] for record in actual_run_files.values()),
        "run total bytes changed")

    results, benchmark_results = verify_results(checker)
    verify_interval_coverage(checker, results)
    verify_legacy(checker, results)
    verify_git_and_environment(checker, manifest)

    checker.check(manifest["results"]["status"] == results["status"],
                  "manifest results status changed")
    checker.check(manifest["results"]["codex_acceptance"] ==
                  results["codex_acceptance"],
                  "manifest Codex status changed")
    checker.check(manifest["results"]["model_cases"] == EXPECTED_CASES,
                  "manifest model case list changed")
    checker.check(manifest["results"]["benchmark_status"] == "PASS" and
                  manifest["results"]["benchmark_checks"] == 151 and
                  manifest["results"]["benchmark_failed"] == 0,
                  "manifest benchmark binding changed")
    checker.check(manifest["results"]["motion_audit_check_count"] ==
                  EXPECTED_MOTION_AUDIT_CHECKS and
                  manifest["results"]["hold_audit_check_count"] ==
                  EXPECTED_HOLD_AUDIT_CHECKS,
                  "manifest audit check counts changed")
    checker.check(manifest["results"]["certificate_leaves_per_motion_case"] ==
                  EXPECTED_CERTIFICATE_LEAVES,
                  "manifest certificate leaves changed")
    checker.check(manifest["results"]["pointwise_leaf_count"] ==
                  results["aggregate"]["pointwise_leaf_count"] and
                  manifest["results"]["pointwise_vehicle_count"] ==
                  results["aggregate"]["pointwise_vehicle_count"],
                  "manifest pointwise counts changed")
    for ratio_name in (
            "max_pointwise_jerk_ratio", "max_pointwise_component_ratio",
            "max_pointwise_p_sss_ratio"):
        checker.check(manifest["results"][ratio_name] ==
                      results["aggregate"][ratio_name],
                      "manifest pointwise ratio changed: " + ratio_name)
    checker.check(manifest["results"]["numerics_sha256"] ==
                  results["interval_coverage"]["numerics_sha256"],
                  "manifest numerics digest changed")
    checker.check(manifest["results"]["degenerate_counterexample_norm"] ==
                  EXPECTED_DEGENERATE_NORM,
                  "manifest degenerate norm changed")
    checker.check(manifest["results"]["narrow_interval_case_count"] == 24 and
                  manifest["results"]["narrow_interval_failure_count"] == 0,
                  "manifest narrow interval binding changed")
    checker.check(manifest["results"]["jerk_overlimit_audit_reason"] ==
                  "DYNAMIC_LIMIT" and
                  manifest["results"]["interior_overlimit_samples"] > 0,
                  "manifest negative jerk binding changed")
    checker.check(manifest["results"]["broad_interval_coarse_reason"] ==
                  "BOUND_UNCONFIRMED" and
                  manifest["results"]["broad_interval_refined_leaves"] > 3,
                  "manifest fail-closed binding changed")
    checker.check(manifest["results"]["legacy_status"] == "FAIL",
                  "manifest legacy status is not truthful")
    checker.check(manifest["results"]["legacy_failure"] ==
                  "timing_continuous",
                  "manifest legacy failure changed")
    checker.check(manifest["results"]["collision_checked"] is False and
                  manifest["results"]["executable"] is False and
                  manifest["results"]["map_used"] is False and
                  manifest["results"]["ros_used"] is False and
                  manifest["results"]["time_optimal"] is False and
                  manifest["results"]["g4_entered"] is False,
                  "manifest scope claims a forbidden result")

    bytecode = [path for path in RUN.rglob("*")
                if path.name == "__pycache__" or
                path.suffix in (".pyc", ".pyo")]
    checker.check(bytecode == [], "generated bytecode exists under the run")
    repo_bytecode = [path for path in REPO_ROOT.rglob("*")
                     if path.name == "__pycache__" and
                     "evaluation" not in path.parts]
    checker.check(repo_bytecode == [],
                  "generated bytecode exists in the source tree")

    if checker.failures:
        print(json.dumps({
            "status": "VERIFY_FAILED",
            "check_count": checker.count,
            "failure_count": len(checker.failures),
            "failures": checker.failures[:20],
        }, indent=2))
        raise SystemExit(1)
    print(json.dumps({
        "status": "VERIFIED",
        "run_id": RUN.name,
        "manifest": str(MANIFEST.relative_to(REPO_ROOT)),
        "manifest_sha256": sha256(MANIFEST),
        "check_count": checker.count,
        "failure_count": 0,
        "source_file_count": manifest["source_file_count"],
        "run_file_count": manifest["run_file_count"],
        "run_total_bytes": manifest["run_total_bytes"],
        "g2_run_files_checked": g2_checked,
        "protected_added": ALLOWED_NEW,
        "protected_changed": ALLOWED_CHANGED,
        "protected_removed": [],
        "protected_tree_sha256":
            manifest["protected"]["current_full_protected_tree_sha256"],
        "model_case_count": len(EXPECTED_CASES),
        "benchmark_check_count": EXPECTED_CHECK_COUNT,
        "test_counts": EXPECTED_TEST_COUNTS,
        "motion_audit_check_count": EXPECTED_MOTION_AUDIT_CHECKS,
        "hold_audit_check_count": EXPECTED_HOLD_AUDIT_CHECKS,
        "pointwise_leaf_count":
            results["aggregate"]["pointwise_leaf_count"],
        "pointwise_vehicle_count":
            results["aggregate"]["pointwise_vehicle_count"],
        "numerics_sha256":
            results["interval_coverage"]["numerics_sha256"],
        "legacy_status": "FAIL_PRE_EXISTING_TIMING_CONTINUOUS",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "scope": {
            "collision_checked": False,
            "executable": False,
            "map_used": False,
            "ros_used": False,
            "time_optimal": False,
            "g4_entered": False,
        },
    }, indent=2))


if __name__ == "__main__":
    main()