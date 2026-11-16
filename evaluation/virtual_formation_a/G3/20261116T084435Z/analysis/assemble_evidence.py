"""Assemble and cross-check the G3 non-finite fix evidence."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[5]
RUN = Path(__file__).resolve().parents[1]
BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
AUDIT = ("catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
         "formation_continuous_a/audit.py")
AUDIT_TEST = "tests/test_virtual_a_audit.py"
TEST_COUNTS = {
    "audit": 64, "timing": 18, "trajectory": 23, "collision": 17,
    "pipeline": 8,
}
EVIDENCE_FILES = (
    "analysis/nonfinite_probe.json",
    "analysis/scene_regression.json",
    "analysis/protection_check.json",
    "environment.json",
    "logs/audit.log",
    "logs/timing.log",
    "logs/trajectory.log",
    "logs/collision.log",
    "logs/pipeline.log",
    "logs/nonfinite_probe.log",
    "logs/scene_regression.log",
    "logs/protection_check.log",
    "logs/environment.log",
    "logs/test_exit_codes.log",
)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load(relative):
    return json.loads((RUN / relative).read_text(encoding="utf-8"))


def require(condition, label):
    if not condition:
        raise SystemExit("ASSEMBLY FAILED: " + label)


def parse_test(name):
    log = (RUN / "logs" / (name + ".log")).read_text(encoding="utf-8")
    match = re.search(r"Ran (\d+) tests? in [0-9.]+s", log)
    require(match is not None, name + " count")
    require(int(match.group(1)) == TEST_COUNTS[name], name + " count value")
    require(log.rstrip().splitlines()[-1].strip() == "OK", name + " OK")
    return {
        "count": int(match.group(1)),
        "log_sha256": sha256(RUN / "logs" / (name + ".log")),
    }


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(ROOT), check=True,
        capture_output=True, text=True).stdout


def main():
    probe = load("analysis/nonfinite_probe.json")
    scene = load("analysis/scene_regression.json")
    protection = load("analysis/protection_check.json")
    environment = load("environment.json")
    require(probe["status"] == "PASS", "probe status")
    require(probe["metric_case_count"] == 30, "probe metric count")
    require(probe["wheel_angle_case_count"] == 3, "probe angle count")
    require(probe["curvature_case_count"] == 3, "probe curvature count")
    require(probe["generation_source_case_count"] == 10,
            "probe generation source count")
    require(probe["mixed_angle_case_count"] == 2,
            "probe mixed angle count")
    require(set(probe["covered_vehicles"]) == {1, 2, 3},
            "probe vehicle coverage")
    require(set(probe["covered_wheels"]) == {0, 1, 2, 3},
            "probe wheel coverage")
    require(len(set(probe["metric_names"])) == 10, "probe metric set")
    require(scene["status"] == "PASS", "scene status")
    require(scene["result"]["status"] == "SUCCESS", "scene result")
    require(scene["result"]["executable"] is True, "scene executable")
    for layer in ("geometry_audit", "dynamic_audit", "collision_audit"):
        require(scene["result"][layer]["passed"] is True, "scene " + layer)
    require(scene["old_g4_benchmark_s1"]["model_sha256_equal"] is True,
            "scene model digest differs from accepted G4")
    require(scene["input_bundle"]["reference_file_present"] is False,
            "reference present in the input bundle")
    require(protection["status"] == "PASS", "protection status")
    require(protection["source_tree"]["changed_source_files"] == [AUDIT,
                                                                 AUDIT_TEST],
            "source change set")
    require(len(protection["source_tree"]["missing_source_files"]) == 0,
            "missing source")
    require(len(protection["historical"]["runs"]) == 4, "historical runs")
    require(environment["git"]["head"] == BASELINE_HEAD, "environment head")
    require(environment["git"]["staged_bytes"] == 0, "staged changes")

    exits = {}
    for line in (RUN / "logs/test_exit_codes.log").read_text(
            encoding="utf-8").splitlines():
        name, value = line.split("=")
        exits[name] = int(value)
    require(exits == {
        "test_" + name: 0 for name in TEST_COUNTS}, "test exit codes")
    command_exits = {}
    for line in (RUN / "logs/command_exit_codes.log").read_text(
            encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("run_id="):
            continue
        name, value = line.split("=")
        command_exits[name] = int(value)
    required_command_exits = {
        "nonfinite_probe": 0,
        "scene_regression": 0,
        "protection_check": 0,
        "environment": 0,
    }
    allowed_command_exits = set(required_command_exits) | {
        "audit_test", "timing_test", "trajectory_test", "collision_test",
        "pipeline_test", "assemble_evidence",
    }
    require(set(command_exits) <= allowed_command_exits,
            "unknown command exit entry")
    for name, expected in required_command_exits.items():
        require(command_exits.get(name) == expected,
                "command exit " + name)
    if "assemble_evidence" in command_exits:
        require(command_exits["assemble_evidence"] == 0,
                "command exit assemble_evidence")

    tests = {name: parse_test(name) for name in TEST_COUNTS}
    head = git("rev-parse", "HEAD").strip()
    branch = git("rev-parse", "--abbrev-ref", "HEAD").strip()
    staged = git("diff", "--cached", "--binary")
    require(head == BASELINE_HEAD, "HEAD")
    require(branch == "feature/virtual-rigid-formation-a", "branch")
    require(staged == "", "staged changes")

    results = {
        "schema": "virtual_rigid_a.g3.nonfinite_fix.results.v1",
        "stage": "G3_NONFINITE_FIX",
        "run_id": RUN.name,
        "status": "PASS_PENDING_CODEX_RECHECK",
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "baseline_head": BASELINE_HEAD,
        "git": {
            "head": head,
            "branch": branch,
            "staged_bytes": len(staged.encode("utf-8")),
            "status_short": git("status", "--short").splitlines(),
            "committed": "NO",
            "pushed": "NO",
            "branch_created": "NO",
        },
        "changed_files": {
            "authorised": [AUDIT, AUDIT_TEST],
            "current_sha256": {
                AUDIT: sha256(ROOT / AUDIT),
                AUDIT_TEST: sha256(ROOT / AUDIT_TEST),
            },
            "accepted_sha256": {
                AUDIT: protection["source_tree"]["accepted_sha256"][AUDIT],
                AUDIT_TEST:
                    protection["source_tree"]["accepted_sha256"][AUDIT_TEST],
            },
            "only_authorised_sources_changed": True,
            "accepted_source_file_count":
                protection["source_tree"]["accepted_source_file_count"],
        },
        "tests": {
            "counts": TEST_COUNTS,
            "total": sum(TEST_COUNTS.values()),
            "all_ok": True,
            "exit_codes": exits,
            "logs": tests,
        },
        "nonfinite_probe": {
            "sha256": sha256(RUN / "analysis/nonfinite_probe.json"),
            "status": probe["status"],
            "metric_case_count": probe["metric_case_count"],
            "wheel_angle_case_count": probe["wheel_angle_case_count"],
            "curvature_case_count": probe["curvature_case_count"],
            "generation_source_case_count":
                probe["generation_source_case_count"],
            "mixed_angle_case_count": probe["mixed_angle_case_count"],
            "covered_vehicles": probe["covered_vehicles"],
            "covered_wheels": probe["covered_wheels"],
            "metric_names": probe["metric_names"],
            "finite_cases": probe["finite_cases"],
        },
        "g4_scene_regression": {
            "sha256": sha256(RUN / "analysis/scene_regression.json"),
            "scene": scene["scene"],
            "status": scene["status"],
            "result_status": scene["result"]["status"],
            "executable": scene["result"]["executable"],
            "checks": scene["result"]["checks"],
            "request_sha256": scene["result"]["request_sha256"],
            "grid_sha256": scene["result"]["grid_sha256"],
            "planning_context_sha256":
                scene["result"]["planning_context_sha256"],
            "model_sha256": scene["result"]["model_sha256"],
            "old_g4_model_sha256_equal":
                scene["old_g4_benchmark_s1"]["model_sha256_equal"],
            "geometry_audit_passed":
                scene["result"]["geometry_audit"]["passed"],
            "dynamic_audit_passed":
                scene["result"]["dynamic_audit"]["passed"],
            "collision_audit_passed":
                scene["result"]["collision_audit"]["passed"],
            "read_guard_active_during_planning": True,
            "reference_file_present": False,
        },
        "protection": {
            "sha256": sha256(RUN / "analysis/protection_check.json"),
            "acceptance": protection["acceptance"],
            "historical_runs": protection["historical"]["runs"],
            "historical_total_checked_files":
                protection["historical"]["total_checked_files"],
            "historical_total_checked_bytes":
                protection["historical"]["total_checked_bytes"],
        },
        "environment": {
            "sha256": sha256(RUN / "environment.json"),
            "python": environment["python"]["version"].split()[0],
            "packages": {
                name: environment["packages"][name].get("version")
                for name in ("numpy", "scipy", "casadi", "yaml")},
        },
        "legacy_timing_regression": {
            "rerun": False,
            "note": (
                "Historical legacy timing_continuous regression was not "
                "rerun.  The pre-existing KeyError 'dynamic_peaks' failure "
                "recorded by G3/G4 remains open and is not claimed passing."),
        },
        "evidence_files": {
            relative: sha256(RUN / relative)
            for relative in EVIDENCE_FILES},
        "nonclaims": [
            "No Codex acceptance is claimed; this run is pending re-review.",
            "The non-finite cases are direct synthetic inputs and a controlled "
            "injection, not naturally occurring motion values.",
            "The G4 S1 regression is one known-feasible A-only frozen scene, "
            "not a success rate, completeness or real-time claim.",
            "The legacy timing_continuous regression was not rerun.",
            "No ROS, Gazebo, hardware or G5 work was performed.",
        ],
    }
    print(json.dumps(results, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()