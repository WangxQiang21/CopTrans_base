"""Stdlib-only independent verification of the G4 source manifest.

Recomputes every source and run-file digest, rechecks the frozen scene inputs
from raw PGM/YAML bytes, rechecks benchmark thresholds and negative-case
coverage, and confirms the G1/G2/G3 acceptance bindings and git state. Imports
nothing from the project and never writes ACCEPTED.
"""
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
MANIFEST_PATH = RUN / "source_manifest.json"
SCENES = ("S1", "S1_mirror", "S2", "S2_mirror", "S3", "S3_mirror")
EXPECTED_BASELINE = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
EXPECTED_SCENE_MANIFEST_FILE_SHA = (
    "3cd821dfe78bf398c2fa84f855f3fb1a628f7c2e1014fc141c9adeb2cfd54516")
EXPECTED_BENCHMARK_SUMMARY_SHA = (
    "d08f28ab82620e2a13f2325b766e776c2e3759f946a2304bde2d11ddd664f86f")
EXPECTED_RESULTS_SHA = (
    "a48b6befabcc28c7b8987af67494477fec84acf8587ac2af2ec15c1f1e035640")
EXPECTED_PROTECTED_TREE = (
    "157686a3832cb0fd94241407866e33443a3b9188483a257fd16510916e78143d")
EXPECTED_SOURCE_COUNT = 70
EXPECTED_RUN_FILE_COUNT_AT_WRITE = None  # checked against the manifest itself
CHECKS = []


def require(condition, message):
    if not condition:
        raise SystemExit("VERIFY FAILED: " + message)
    CHECKS.append(message)


def sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical(value, ensure_ascii=False):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"),
        ensure_ascii=ensure_ascii, allow_nan=False)


def next_up(value):
    value = float(value)
    if math.isnan(value) or math.isinf(value):
        return value
    if value == 0.0:
        return 5e-324
    bits = struct.unpack("<Q", struct.pack("<d", value))[0]
    bits = bits + 1 if value > 0.0 else bits - 1
    return struct.unpack("<d", struct.pack("<Q", bits))[0]


def read_pgm(path):
    data = Path(path).read_bytes()
    require(data.startswith(b"P5"), "PGM {} is P5".format(Path(path).name))
    index = 2
    fields = []
    while len(fields) < 3:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if index < len(data) and data[index:index + 1] == b"#":
            while index < len(data) and data[index:index + 1] not in (
                    b"\n", b"\r"):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        fields.append(data[start:index])
    while index < len(data) and data[index:index + 1].isspace():
        index += 1
    width, height, maxval = (int(field) for field in fields)
    pixels = data[index:index + width * height]
    require(len(pixels) == width * height,
            "PGM {} raster complete".format(Path(path).name))
    return width, height, maxval, pixels


def parse_yaml(path):
    metadata = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        key, _, value = line.partition(":")
        metadata[key.strip()] = value.strip()
    return {
        "image": metadata["image"].strip('"\''),
        "resolution": float(metadata["resolution"]),
        "origin": tuple(float(part.strip()) for part in
                        metadata["origin"].strip("[]").split(",")),
        "negate": int(metadata.get("negate", "0")),
        "occupied_thresh": float(metadata.get("occupied_thresh", "0.65")),
        "free_thresh": float(metadata.get("free_thresh", "0.196")),
    }


def map_signature(yaml_path):
    metadata = parse_yaml(yaml_path)
    width, height, maxval, pixels = read_pgm(
        Path(yaml_path).parent / metadata["image"])
    rows = []
    for image_row in range(height - 1, -1, -1):
        start = image_row * width
        row = []
        for pixel in pixels[start:start + width]:
            probability = (pixel / float(maxval)) if metadata["negate"] else \
                ((float(maxval) - pixel) / float(maxval))
            row.append(0 if probability < metadata["free_thresh"] else 1)
        rows.append(row)
    flat = bytes(value for row in rows for value in row)
    summary = {
        "height": height,
        "width": width,
        "resolution_m": metadata["resolution"],
        "origin_x": metadata["origin"][0],
        "origin_y": metadata["origin"][1],
        "occupancy_sha256": sha256_bytes(flat),
        "cell_count": height * width,
        "occupied_cells": sum(flat),
    }
    return sha256_bytes(canonical(summary).encode("utf-8")), metadata


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def verify_source_files(manifest):
    require(manifest["schema"] == "virtual_rigid_a.g4.source_manifest.v1",
            "manifest schema")
    require(manifest["stage"] == "G4", "manifest stage")
    require(manifest["run_id"] == RUN.name, "manifest run id")
    require(manifest["baseline_head"] == EXPECTED_BASELINE,
            "manifest baseline")
    require(manifest["codex_acceptance"] == "NOT_RUN_NOT_CLAIMED",
            "manifest Codex nonclaim")
    sources = manifest["source_files"]
    require(len(sources) == EXPECTED_SOURCE_COUNT,
            "source file count 70")
    require(len(manifest["run_files"]) ==
            manifest["run_file_count"], "run file count self-consistent")
    missing = []
    mismatched = []
    for relative, record in sorted(sources.items()):
        path = REPO_ROOT / relative
        if not path.is_file():
            missing.append(relative)
            continue
        if sha256_file(path) != record["sha256"] or \
                path.stat().st_size != record["bytes"]:
            mismatched.append(relative)
    require(not missing, "every source file exists")
    require(not mismatched, "every source digest and size matches")
    new_files = manifest["g4_new_source_files"]
    require(len(new_files) == 39, "39 G4 new source/fixture files")
    require(len(manifest["g4_new_code_and_test_files"]) == 7,
            "7 G4 new code/test/tool/doc files")
    require(len(manifest["g4_new_fixture_files"]) == 32,
            "32 G4 new fixtures")
    for relative in new_files:
        require(relative in sources, "G4 new source listed: " + relative)
    changed = manifest["changed_accepted_source_list"]
    require(changed == sorted(changed), "changed list sorted")
    require(len(changed) == 5, "five accepted G3 sources changed")
    for relative in changed:
        record = manifest["changed_accepted_sources"][relative]
        require(record["changed"] is True,
                "changed flag true: " + relative)
        require(record["g4_sha256"] == sources[relative]["sha256"],
                "changed digest bound: " + relative)
    for relative, record in manifest["changed_accepted_sources"].items():
        if relative not in changed:
            require(record["changed"] is False,
                    "unchanged source flag: " + relative)
            require(record["g3_sha256"] == record["g4_sha256"],
                    "unchanged source digest: " + relative)


def verify_run_files(manifest):
    actual = set()
    for path in sorted(RUN.rglob("*")):
        if path.is_file():
            actual.add(path.relative_to(RUN).as_posix())
    excluded = set(manifest["excluded_self_referential"])
    listed = set(manifest["run_files"])
    extra = sorted(actual - listed - excluded)
    missing = sorted(listed - actual)
    require(not extra, "no unlisted run files")
    require(not missing, "no missing run files")
    for relative, record in manifest["run_files"].items():
        path = RUN / relative
        require(sha256_file(path) == record["sha256"],
                "run file digest: " + relative)
        require(path.stat().st_size == record["bytes"],
                "run file size: " + relative)
    require(sha256_file(RUN / "source_manifest.json") != "", "manifest exists")


def verify_scene_freeze(manifest):
    scene_manifest_path = RUN / "scenes/scene_manifest.json"
    require(sha256_file(scene_manifest_path) ==
            EXPECTED_SCENE_MANIFEST_FILE_SHA, "scene manifest file digest")
    require(sha256_file(scene_manifest_path) ==
            manifest["scene_freeze"]["manifest_sha256"],
            "scene manifest bound by source manifest")
    document = load(scene_manifest_path)
    declared = document.pop("manifest_sha256")
    require(declared ==
            manifest["scene_freeze"]["manifest_self_digest"],
            "scene manifest self digest bound")
    require(sha256_bytes(canonical(document, ensure_ascii=True).encode(
        "utf-8")) == declared, "scene manifest self digest recomputed")
    require(document["frozen"] is True, "scene manifest frozen")
    require(document["reference_verified"] is True, "references verified")
    require(document["inputs_have_no_reference"] is True,
            "inputs claim no reference")
    require(document["failures"] == [], "scene failures empty")
    require(sorted(document["scenes"]) == sorted(SCENES), "scene set")
    require(len(document["reference_verification"]) == 6,
            "six reference verifications")
    for record in document["reference_verification"]:
        require(record["passed"] is True,
                "reference verification passed: " + record["scene_id"])
        require(all(record["checks"].values()),
                "reference subchecks: " + record["scene_id"])
    for name in SCENES:
        entry = document["scenes"][name]
        require(entry["status"] == "REFERENCE_PASSED",
                "reference status: " + name)
        input_dir = RUN / "scenes" / entry["input_dir"]
        files = sorted(path.name for path in input_dir.iterdir()
                       if path.is_file())
        require(files == ["map.pgm", "map.yaml", "request.json"],
                "reference-free input bundle: " + name)
        signature, metadata = map_signature(input_dir / "map.yaml")
        require(signature == entry["map_sha256"],
                "raw map signature: " + name)
        require(metadata["resolution"] == document["resolution_m"],
                "scene resolution: " + name)
        payload = load(input_dir / "request.json")
        require(payload["schema"] ==
                "virtual_rigid_a.g4.planning_context.v1",
                "planning payload schema: " + name)
        require(payload["stage"] == "G4", "payload stage: " + name)
        require(payload["planner_config"] == document["planner_config"],
                "payload planner config: " + name)
        require(payload["timing_config"] == document["timing_config"],
                "payload timing config: " + name)
        require(payload["collision_config"] == document["collision_config"],
                "payload collision config: " + name)
        request = payload["request"]
        require(request["initial"]["stationary"] is True,
                "initial stationary: " + name)
        require(request["initial"]["wheels_straight"] is True,
                "initial wheels straight: " + name)
        require(request["goal_policy"] == "FREE", "free terminal: " + name)
        reference = load(RUN / "scenes" / entry["reference_file"])
        require(reference["status"] == "REFERENCE_PASSED",
                "reference file status: " + name)
        require(reference["scene_id"] == name, "reference scene id: " + name)
        require(reference["independent"]["accepted"] is True,
                "reference independently accepted: " + name)
        for layer in ("geometry_audit", "dynamic_audit", "collision_audit"):
            require(reference[layer]["passed"] is True,
                    "reference {} passed: {} ".format(layer, name))


def verify_benchmark(manifest):
    summary_path = RUN / "benchmark/summary.json"
    require(sha256_file(summary_path) == EXPECTED_BENCHMARK_SUMMARY_SHA,
            "benchmark summary digest")
    summary = load(summary_path)
    require(summary["status"] == "PASS", "benchmark status PASS")
    require(len(summary["checks"]) == 72, "benchmark 72 checks")
    require(all(record["pass"] is True for record in summary["checks"]),
            "benchmark all checks pass")
    require(summary["codex_acceptance"] == "NOT_RUN_NOT_CLAIMED",
            "benchmark Codex nonclaim")
    require(summary["manifest_sha256"] ==
            manifest["scene_freeze"]["manifest_self_digest"],
            "benchmark binds scene manifest")
    results = load(RUN / "benchmark/results.json")
    require(sorted(results) == sorted(SCENES), "benchmark result scenes")
    for name in SCENES:
        record = results[name]
        require(record["status"] == "SUCCESS", "SUCCESS: " + name)
        require(record["executable"] is True, "executable: " + name)
        require(record["manifest_map_match"] is True,
                "manifest map match: " + name)
        require(record["manifest_request_match"] is True,
                "manifest request match: " + name)
        require(record["reference_free_bundle"] is True,
                "reference-free bundle: " + name)
        require(record["endpoint_error_m"] <= 0.005,
                "endpoint <= 5 mm: " + name)
        require(record["heading_error_rad"] <=
                math.radians(0.2), "heading <= 0.2 deg: " + name)
        require(record["rigidity_error_m"] <= 1e-6,
                "rigidity <= 1e-6 m: " + name)
        require(record["collision_margin_m"] >= 0.05,
                "margin >= 0.05 m: " + name)
        total = 0
        for layer in ("geometry", "dynamics", "collision"):
            audit = record["audits"][layer]
            require(audit["passed"] is True, "{} audit: {}".format(
                layer, name))
            require(audit["request_sha256"] == record["request_sha256"],
                    "{} request binding: {}".format(layer, name))
            require(audit["grid_sha256"] == record["grid_sha256"],
                    "{} grid binding: {}".format(layer, name))
            require(audit["planning_context_sha256"] ==
                    record["planning_context_sha256"],
                    "{} context binding: {}".format(layer, name))
            total += audit["check_count"]
        require(record["checks"] == total, "check sum: " + name)
        require(sha256_bytes(record["model_json"].encode("utf-8")) ==
                record["model_sha256"], "model digest: " + name)
        model = json.loads(record["model_json"])
        require(model["request_sha256"] == record["request_sha256"],
                "model request: " + name)
        require(model["planning_context"]["grid"] ==
                record["model_summary"]["grid"],
                "model context grid summary: " + name)
        envelope = record["audits"]["collision"]["metrics"]["margin"]
        maximum = max(item["envelope_m"]
                      for item in envelope["vehicles"])
        require(maximum == envelope["maximum_envelope_m"],
                "maximum envelope: " + name)
        require(envelope["margin_used_m"] >=
                0.05 + envelope["maximum_envelope_m"],
                "margin dominates envelope: " + name)
        for item in envelope["vehicles"]:
            recomputed = next_up(
                item["eps_position_m"] +
                next_up((item["offset_m"] +
                         item["inflated_body_radius_upper_m"]) *
                        item["eps_formation_heading_rad"]) +
                next_up(item["inflated_body_radius_upper_m"] *
                        item["eps_relative_heading_rad"]))
            require(recomputed == item["envelope_m"],
                    "envelope arithmetic: " + name)
        utilization = record["audits"]["dynamics"]["metrics"][
            "max_utilization"]
        require(max(utilization.values()) <= 1.0,
                "dynamic utilization <= 1: " + name)
        require(record["audits"]["dynamics"]["metrics"][
            "whole_domain_certificate"] is True,
            "whole-domain certificate: " + name)
        pointwise = record["audits"]["dynamics"]["metrics"][
            "pointwise_interval_checks"]
        require(pointwise["max_component_ratio"] <= 1.0,
                "pointwise component <= 1: " + name)
        require(pointwise["max_jerk_ratio"] <= 1.0,
                "pointwise jerk <= 1: " + name)
        require(pointwise["max_p_sss_ratio"] <= 1.0,
                "pointwise p_sss <= 1: " + name)
        require(len(record["attempts"]) > 0, "attempts retained: " + name)
        require(len(record["routes"]) <= 3, "at most three routes: " + name)


def verify_results_and_logs(manifest):
    results_path = RUN / "results.json"
    require(sha256_file(results_path) == EXPECTED_RESULTS_SHA,
            "results.json digest")
    results = load(results_path)
    require(results["status"] == "PASS_PENDING_CODEX_RECHECK",
            "results pending Codex")
    require(results["codex_acceptance"] == "NOT_RUN_NOT_CLAIMED",
            "results Codex nonclaim")
    require(len(results["scenes"]) == 6, "results six scenes")
    require(all(record["executable"] for record in
                results["scenes"].values()), "results all executable")
    require(results["attempt_summary"]["all_attempts_retained"] is True,
            "all attempts retained")
    require(results["attempt_summary"]["total_attempts"] == 48,
            "48 attempts recorded")
    require([case["case"] for case in
             results["numbered_negative_coverage"]] == list(range(1, 8)),
            "negative cases 1..7")
    require(all(case["observed_passing"] for case in
                results["numbered_negative_coverage"]),
            "all numbered negatives observed")
    require(results["tests"]["total_tests"] == 179,
            "179 tests total")
    require({record["stage"]: record["checks"] for record in
             results["regression_benchmarks"]} ==
            {"G1": 26, "G2": 124, "G3": 151},
            "G1/G2/G3 regression checks")
    require(sha256_file(RUN / "analysis/g4_numerics.json") ==
            results["independent_numerics"]["sha256"],
            "numerics digest bound")
    require(sha256_file(RUN / "environment.json") ==
            results["environment"]["sha256"], "environment digest bound")

    collision = (RUN / "logs/collision.log").read_text(encoding="utf-8")
    pipeline = (RUN / "logs/pipeline.log").read_text(encoding="utf-8")
    require("Ran 11 tests" in collision and "\nOK\n" in collision,
            "collision suite 11 OK")
    require("Ran 8 tests" in pipeline and "\nOK\n" in pipeline,
            "pipeline suite 8 OK")
    regression = (RUN / "logs/g1_g2_g3_regression.log").read_text(
        encoding="utf-8")
    for count in (20, 26, 22, 51, 18, 23):
        require("Ran {} tests".format(count) in regression,
                "regression suite count {}".format(count))
    require(regression.count("\nOK\n") == 6,
            "six regression suites OK")
    texts = {"logs/collision.log": collision,
             "logs/pipeline.log": pipeline}
    for case in results["numbered_negative_coverage"]:
        for name in case["tests"]:
            observed = any(
                any(line.startswith(name + " ") and
                    line.rstrip().endswith("... ok")
                    for line in texts[log].splitlines())
                for log in texts)
            require(observed, "negative test passed: " + name)


def verify_protection(manifest):
    path = RUN / "analysis/protection_diff_vs_g3_accepted.json"
    require(sha256_file(path) == manifest["protected"]["comparison_sha256"],
            "protection diff digest")
    protection = load(path)
    require(protection["current"]["full_protected_tree_sha256"] ==
            EXPECTED_PROTECTED_TREE, "protected tree digest")
    require(protection["authorization"]["only_authorized_changes"] is True,
            "only authorized protected changes")
    require(protection["current"]["removed_count"] == 0,
            "no protected removals")
    added = sorted(row["path"] for row in
                   protection["added_vs_g3_accepted"])
    changed = sorted(row["path"] for row in
                     protection["changed_vs_g3_accepted"])
    require(added == sorted(manifest["protected"][
        "added_vs_g3_accepted"]), "added list bound")
    require(changed == sorted(manifest["protected"][
        "changed_vs_g3_accepted"]), "changed list bound")
    require(len(added) == 38, "38 protected additions")
    require(len(changed) == 5, "5 protected changes")
    require(protection["g3_evidence"]["protected_tree_sha256"] ==
            "9bee7d7a483418877277fe63c83986363f7ffc114f03db6ff265927092d89fba",
            "accepted G3 tree bound")
    acceptance = load(REPO_ROOT /
                     "evaluation/virtual_formation_a/G3/"
                     "codex_acceptance_rework_01/CODEX_ACCEPTANCE.json")
    require(acceptance["status"] == "ACCEPTED", "G3 acceptance ACCEPTED")
    require(acceptance["next_stage_authorized"] is False,
            "G3 acceptance did not authorize G4")
    require(acceptance["source_manifest_sha256"] ==
            "bad6f55441bd34ca8d164cc954170336b56df398fcb0912dae10a4588279ceea",
            "G3 acceptance manifest digest")
    g3_manifest_path = REPO_ROOT / (
        "evaluation/virtual_formation_a/G3/20261116T031558Z/"
        "source_manifest.json")
    require(sha256_file(g3_manifest_path) ==
            acceptance["source_manifest_sha256"],
            "accepted G3 manifest unchanged")
    g3_manifest = load(g3_manifest_path)
    excluded = set(g3_manifest["excluded_self_referential"])
    for relative, record in g3_manifest["run_files"].items():
        if relative in excluded:
            continue
        require(sha256_file(g3_manifest_path.parent / relative) ==
                record["sha256"], "G3 run file unchanged")
    for relative, digest in acceptance["source_sha256"].items():
        require(g3_manifest["source_files"][relative]["sha256"] == digest,
                "G3 accepted source unchanged")
    require(sha256_file(REPO_ROOT /
            "evaluation/virtual_formation_a/G1/20261115T152858Z/"
            "legacy/protected_after.json") == manifest["protected"][
                "g1_protected_after_sha256"], "G1 snapshot unchanged")
    require(sha256_file(REPO_ROOT /
            "evaluation/virtual_formation_a/G2/"
            "codex_acceptance_rework_03/CODEX_ACCEPTANCE.json") ==
            manifest["protected"]["g2_acceptance_sha256"],
            "G2 acceptance unchanged")
    require(sha256_file(REPO_ROOT /
            "evaluation/virtual_formation_a/G2/20261115T174526Z/"
            "source_manifest.json") == manifest["protected"][
                "g2_manifest_sha256"], "G2 manifest unchanged")


def verify_git(manifest):
    def run(*args):
        return subprocess.run(
            ["git"] + list(args), cwd=str(REPO_ROOT), check=True,
            capture_output=True, text=True).stdout
    require(run("rev-parse", "HEAD").strip() ==
            manifest["git"]["head"], "git HEAD unchanged")
    require(run("rev-parse", "--abbrev-ref", "HEAD").strip() ==
            manifest["git"]["branch"], "git branch unchanged")
    require(run("rev-parse", "HEAD").strip() == EXPECTED_BASELINE,
            "HEAD at baseline")
    require(not run("diff", "--cached", "--binary"),
            "no staged changes")
    require(subprocess.run(
        ["git", "merge-base", "--is-ancestor", EXPECTED_BASELINE, "HEAD"],
        cwd=str(REPO_ROOT), check=False).returncode == 0,
        "baseline is ancestor")
    require(manifest["git"]["committed"] == "NO" and
            manifest["git"]["pushed"] == "NO", "no commit or push claimed")


def main():
    require(MANIFEST_PATH.is_file(), "source manifest exists")
    manifest = load(MANIFEST_PATH)
    verify_source_files(manifest)
    verify_run_files(manifest)
    verify_scene_freeze(manifest)
    verify_benchmark(manifest)
    verify_results_and_logs(manifest)
    verify_protection(manifest)
    verify_git(manifest)
    result = {
        "status": "VERIFIED",
        "checks": len(CHECKS),
        "run_id": RUN.name,
        "source_manifest_sha256": sha256_file(MANIFEST_PATH),
        "source_file_count": len(manifest["source_files"]),
        "run_file_count": len(manifest["run_files"]),
        "protected_tree_sha256": EXPECTED_PROTECTED_TREE,
        "scene_manifest_sha256": EXPECTED_SCENE_MANIFEST_FILE_SHA,
        "benchmark_summary_sha256": EXPECTED_BENCHMARK_SUMMARY_SHA,
        "results_sha256": EXPECTED_RESULTS_SHA,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()