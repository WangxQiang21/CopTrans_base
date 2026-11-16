"""Re-run one frozen, reference-free G4 scene end to end after the G3 fix.

The inputs are the S1 bundle frozen by the accepted G4 rework run
``20261116T081458Z``: only ``map.pgm``, ``map.yaml`` and ``request.json``.
The accepted scene manifest is read for expected digests only; a read guard
is active while ``plan_virtual_a`` runs, proving the planner consumes only the
in-memory map/request and no reference, manifest or old audit.
"""
import hashlib
import json
import math
from pathlib import Path
from unittest import mock

from fwis_v3_optimizer.formation_continuous_a.collision import load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    collision_config_from_dict,
    planner_config_from_dict,
    request_fingerprint,
    request_from_payload,
    timing_config_from_dict,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a

ROOT = Path(__file__).resolve().parents[5]
RUN = Path(__file__).resolve().parents[1]
G4_RUN = ROOT / "evaluation/virtual_formation_a/G4/20261116T081458Z"
SCENE = "S1"
INPUT_DIR = G4_RUN / "scenes/inputs" / SCENE
SCENE_MANIFEST = G4_RUN / "scenes/scene_manifest.json"
G4_RESULTS = G4_RUN / "benchmark/results.json"
INPUT_FILES = ("map.pgm", "map.yaml", "request.json")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, label):
    if not condition:
        raise SystemExit("SCENE REGRESSION FAILED: " + label)


def load_bundle():
    names = sorted(path.name for path in INPUT_DIR.iterdir() if path.is_file())
    require(names == list(INPUT_FILES), "input bundle is not map/request only")
    payload = json.loads(
        (INPUT_DIR / "request.json").read_text(encoding="utf-8"))
    require(set(payload) == {
        "schema", "stage", "request", "planner_config", "timing_config",
        "collision_config", "map", "note"}, "request payload keys")
    grid, metadata = load_map(INPUT_DIR / "map.yaml")
    request = request_from_payload(payload["request"])
    planner = planner_config_from_dict(payload["planner_config"])
    timing = timing_config_from_dict(payload["timing_config"])
    collision = collision_config_from_dict(payload["collision_config"])
    from fwis_v3_optimizer.formation_continuous_a.contracts import (
        VirtualAMapRequest,
    )
    map_request_value = VirtualAMapRequest(
        request, grid, planner, timing, collision)
    return payload, grid, metadata, map_request_value


def audit_record(report):
    return {
        "passed": report.passed,
        "reason": report.reason,
        "detail": report.detail,
        "first_failure": report.first_failure,
        "check_count": report.check_count,
    }


def main():
    manifest = json.loads(SCENE_MANIFEST.read_text(encoding="utf-8"))
    expected = manifest["scenes"][SCENE]
    payload, grid, metadata, map_request_value = load_bundle()
    request = map_request_value.request

    request_sha = request_fingerprint(request)
    require(request_sha == expected["request_sha256"], "request digest")
    require(metadata["grid_sha256"] == expected["map_sha256"],
            "grid digest")

    with mock.patch.object(
            Path, "read_text",
            side_effect=AssertionError("unplanned file read")), \
            mock.patch.object(
                Path, "read_bytes",
                side_effect=AssertionError("unplanned file read")), \
            mock.patch.object(
                json, "load",
                side_effect=AssertionError("unplanned JSON load")):
        result = plan_virtual_a(map_request_value)

    require(result.status == "SUCCESS", "status is not SUCCESS")
    require(result.executable is True, "not executable")
    require(result.geometry_audit.passed is True, "geometry audit")
    require(result.dynamic_audit.passed is True, "dynamic audit")
    require(result.collision_audit.passed is True, "collision audit")
    require(result.request_sha256 == request_sha, "result request digest")
    require(result.grid_sha256 == expected["map_sha256"], "result grid digest")
    require(result.planning_context_sha256 ==
            expected["planning_context_sha256"], "result context digest")
    require(result.plan is not None and result.path is not None, "no plan")
    require(result.plan.request_sha256 == result.request_sha256,
            "plan request binding")
    require(result.plan.planning_context_sha256() ==
            result.planning_context_sha256, "plan context binding")
    require(result.plan.model_sha256() == result.model_sha256,
            "plan model binding")
    require(result.collision_audit.grid_sha256 == result.grid_sha256,
            "collision grid binding")
    require(result.checks > 0 and result.collision_audit.check_count > 0,
            "empty check count")
    require(result.collision_audit.metrics.get("map_used") is True,
            "collision map not used")
    endpoint = result.path.endpoint()
    goal = request.goal_qf
    endpoint_error = math.hypot(endpoint.x - goal.x, endpoint.y - goal.y)
    heading_error = abs(math.atan2(
        math.sin(endpoint.psi - goal.psi),
        math.cos(endpoint.psi - goal.psi)))
    require(endpoint_error <= 0.005, "endpoint error")
    require(heading_error <= 0.003490658503988659, "heading error")

    old_results = json.loads(G4_RESULTS.read_text(encoding="utf-8"))
    old = old_results[SCENE]
    output = {
        "schema": "virtual_rigid_a.g3.g4_scene_regression.v1",
        "status": "PASS",
        "run_id": RUN.name,
        "scene": SCENE,
        "input_bundle": {
            "path": str(INPUT_DIR.relative_to(ROOT)),
            "files": {
                name: sha256(INPUT_DIR / name)
                for name in INPUT_FILES},
            "reference_file_present": any(
                "reference" in path.name.lower()
                for path in INPUT_DIR.iterdir()),
            "read_guard_active_during_planning": True,
            "note": payload["note"],
        },
        "expected_from_accepted_g4_manifest": {
            "map_sha256": expected["map_sha256"],
            "request_sha256": expected["request_sha256"],
            "planning_context_sha256": expected["planning_context_sha256"],
            "timed_model_sha256": expected["timed_model_sha256"],
        },
        "result": {
            "status": result.status,
            "executable": result.executable,
            "checks": result.checks,
            "request_sha256": result.request_sha256,
            "grid_sha256": result.grid_sha256,
            "planning_context_sha256": result.planning_context_sha256,
            "model_sha256": result.model_sha256,
            "plan_model_sha256": result.plan.model_sha256(),
            "plan_request_sha256": result.plan.request_sha256,
            "plan_planning_context_sha256":
                result.plan.planning_context_sha256(),
            "path_length_m": result.path.length_m,
            "endpoint_error_m": endpoint_error,
            "heading_error_rad": heading_error,
            "geometry_audit": audit_record(result.geometry_audit),
            "dynamic_audit": audit_record(result.dynamic_audit),
            "collision_audit": audit_record(result.collision_audit),
            "collision_margin_m": result.metrics.get("collision_margin_m"),
            "routes": len(result.routes),
            "rounds": len(result.search["rounds"]),
            "wall_time_s": result.wall_time_s,
            "timings": dict(result.timings),
        },
        "old_g4_benchmark_s1": {
            "status": old["status"],
            "model_sha256": old["model_sha256"],
            "checks": old["checks"],
            "model_sha256_equal": old["model_sha256"] == result.model_sha256,
        },
        "note": (
            "In-memory frozen S1 inputs; planning ran behind a read guard. "
            "This is one known-feasible A-only scene, not a success-rate or "
            "general-completeness claim."),
    }
    print(json.dumps(output, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()