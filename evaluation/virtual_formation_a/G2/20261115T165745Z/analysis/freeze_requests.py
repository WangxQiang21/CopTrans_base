"""Freeze the G2 request/config fixtures and prove the solved requests bind them.

This evidence builder copies the two independent fixture files byte-for-byte,
reconstructs every request from the frozen base and per-request record, and
compares the resulting payload and fingerprint with the benchmark's recorded
production input.  It also proves the frozen planner configuration is exactly
``PlannerConfig()`` (and the audit configuration is ``GeometryAuditConfig()``).
It never feeds the existence reference to the production solver.
"""
import datetime
import hashlib
import json
from pathlib import Path
import shutil

from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GeometryAuditConfig,
    GoalPolicy,
    InitialAssumption,
    PlannerConfig,
    VirtualARequest,
    request_fingerprint,
    request_summary,
)

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
BENCHMARK_RESULTS = RUN / "benchmark" / "results.json"
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures" / "virtual_a"
SOURCES = {
    "open_requests.json": FIXTURE_DIR / "open_requests.json",
    "existence_reference.json": FIXTURE_DIR / "existence_reference.json",
}
NAMES = ("E1", "E2", "E3", "E4", "E5", "E6",
         "STRAIGHT_LINE", "STRAIGHT_POLICY")
EXPECTED_QUADRATIC = {
    "E1": (6.0, 0.1), "E2": (6.0, -0.1), "E3": (6.0, 0.2),
    "E4": (6.0, -0.2), "E5": (8.0, 0.3), "E6": (8.0, -0.3),
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def model_from_fixture(base):
    return FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in base["model"]["offsets"]])


def shapes_from_fixture(base):
    return {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in base["model"]["vehicle_shapes"]}


def request_from_fixture(document, name):
    base = document["base"]
    record = document["requests"][name]
    goal = record["goal_qf"]
    return VirtualARequest(
        plan_id=record["plan_id"],
        formation_id=record["formation_id"],
        model=model_from_fixture(base),
        vehicle_shapes=shapes_from_fixture(base),
        start_qf=FormationPose(**base["start_qf"]),
        start_headings={
            int(key): float(value)
            for key, value in base["start_headings"].items()},
        goal_qf=FormationPose(goal["x"], goal["y"], goal["psi"]),
        initial=InitialAssumption(**base["initial"]),
        goal_policy=GoalPolicy(record["goal_policy"]),
        limits=FormationTimingLimits(**base["limits"]),
    )


def main():
    frozen_dir = RUN / "frozen"
    frozen_dir.mkdir(parents=True, exist_ok=True)
    records = {}
    for name, source in SOURCES.items():
        target = frozen_dir / name
        shutil.copyfile(str(source), str(target))
        source_hash = sha256(source)
        copy_hash = sha256(target)
        if source_hash != copy_hash:
            raise SystemExit("frozen copy differs for " + name)
        records[name] = {
            "source_path": source.relative_to(REPO_ROOT).as_posix(),
            "source_sha256": source_hash,
            "frozen_path": target.relative_to(RUN).as_posix(),
            "frozen_sha256": copy_hash,
            "bytes": target.stat().st_size,
            "byte_identical": True,
        }

    requests_document = json.loads(
        (frozen_dir / "open_requests.json").read_text(encoding="utf-8"))
    reference_document = json.loads(
        (frozen_dir / "existence_reference.json").read_text(encoding="utf-8"))
    benchmark = json.loads(BENCHMARK_RESULTS.read_text(encoding="utf-8"))
    if sorted(benchmark) != sorted(NAMES):
        raise SystemExit("benchmark request set is not the frozen set")

    default_config = PlannerConfig()
    frozen_config = dict(requests_document["planner_config"])
    reconstructed_config = PlannerConfig(**frozen_config)
    if reconstructed_config != default_config:
        raise SystemExit("frozen planner config is not the default")
    if frozen_config != default_config.as_dict():
        raise SystemExit("frozen planner config dictionary differs from "
                         "PlannerConfig().as_dict()")
    if requests_document["schema"] != "virtual_rigid_a.g2.open_requests.v1":
        raise SystemExit("unexpected open-request schema")
    if reference_document["schema"] != \
            "virtual_rigid_a.g2.existence_reference.v1":
        raise SystemExit("unexpected existence-reference schema")
    if reference_document["not_read_by_production"] is not True:
        raise SystemExit("existence reference is not marked independent")

    request_records = []
    for name in NAMES:
        request = request_from_fixture(requests_document, name)
        summary = request_summary(request)
        fingerprint = request_fingerprint(request)
        recorded = benchmark[name]
        if recorded["request"] != summary:
            raise SystemExit("production request differs from frozen fixture "
                             "for " + name)
        if recorded["request_sha256"] != fingerprint:
            raise SystemExit("request fingerprint differs for " + name)
        request_records.append({
            "name": name,
            "plan_id": request.plan_id,
            "formation_id": request.formation_id,
            "goal_policy": request.goal_policy.value,
            "request_sha256": fingerprint,
            "benchmark_request_summary_matches_frozen": True,
            "request_summary_sha256": hashlib.sha256(
                json.dumps(summary, sort_keys=True, separators=(",", ":"),
                           ensure_ascii=False).encode("utf-8")).hexdigest(),
        })

    quadratic = reference_document["quadratic_profiles"]
    if sorted(quadratic) != sorted(EXPECTED_QUADRATIC):
        raise SystemExit("quadratic reference set changed")
    for name, (S, alpha) in EXPECTED_QUADRATIC.items():
        if float(quadratic[name]["S"]) != S:
            raise SystemExit("reference S changed for " + name)
        if float(quadratic[name]["alpha"]) != alpha:
            raise SystemExit("reference alpha changed for " + name)
        if quadratic[name]["sampled_C_within_limit"] is not True:
            raise SystemExit("reference curvature was not independently "
                             "checked for " + name)
        goal = requests_document["requests"][name]["goal_qf"]
        exact = quadratic[name]["exact_endpoint"]
        if abs(exact["x"] - goal["x"]) > 1e-9 or \
                abs(exact["y"] - goal["y"]) > 1e-9 or \
                abs(exact["psi"] - goal["psi"]) > 1e-12:
            raise SystemExit("reference endpoint differs from request " + name)

    straight = reference_document["straight_spline"]
    if not str(straight["bc_type"]).startswith("clamped"):
        raise SystemExit("straight reference is not clamped")
    if straight["spline"] != "scipy.interpolate.CubicSpline":
        raise SystemExit("straight reference is not a clamped CubicSpline")
    if [float(v) for v in straight["stations_m"]] != [
            0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]:
        raise SystemExit("straight reference stations changed")
    if [float(v) for v in straight["curvature_1_per_m"]] != [
            0.0, 0.10, 0.20, 0.0, -0.20, -0.10, 0.0]:
        raise SystemExit("straight reference curvature changed")
    if any(float(v) != 0.0 for v in straight["slope_1_per_m2"][::6]):
        raise SystemExit("straight reference endpoint slopes are nonzero")
    straight_goal = requests_document["requests"]["STRAIGHT_POLICY"]["goal_qf"]
    if abs(straight["exact_endpoint"]["x"] - straight_goal["x"]) > 1e-9 or \
            abs(straight["exact_endpoint"]["y"] - straight_goal["y"]) > 1e-9:
        raise SystemExit("straight reference endpoint differs from request")

    audit_config = GeometryAuditConfig()
    manifest = {
        "schema": "virtual_rigid_a.g2.frozen_fixtures.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "files": records,
        "planner_config": {
            "frozen_values": frozen_config,
            "default_values": default_config.as_dict(),
            "reconstructed_equals_PlannerConfig": (
                reconstructed_config == default_config),
            "frozen_dict_equals_default_dict": (
                frozen_config == default_config.as_dict()),
            "frozen_sha256_canonical": hashlib.sha256(
                json.dumps(frozen_config, sort_keys=True,
                           separators=(",", ":")).encode("utf-8")).hexdigest(),
        },
        "audit_config": {
            "default_values": audit_config.as_dict(),
            "default_sha256_canonical": hashlib.sha256(
                json.dumps(audit_config.as_dict(), sort_keys=True,
                           separators=(",", ":")).encode("utf-8")).hexdigest(),
            "used_by_production_solve": False,
        },
        "requests": request_records,
        "request_count": len(request_records),
        "solve_inputs": {
            "call_signature": "solve_open_path(request, config)",
            "request_source": "frozen/open_requests.json base + request record",
            "config_source": "frozen PlannerConfig() default",
            "reference_profiles_passed_to_solve": False,
            "existence_reference_role": (
                "independent audit/oracle only; never a solver seed, warm "
                "start, cache, lookup table or production input"),
            "isolation_test": (
                "test_virtual_a_optimizer.InitialGuessTests."
                "test_solve_reads_no_reference_or_fixture_file"),
        },
        "existence_reference": {
            "quadratic_profiles": {
                name: {
                    "S": quadratic[name]["S"],
                    "alpha": quadratic[name]["alpha"],
                    "formula": quadratic[name]["formula"],
                    "sampled_max_abs_C_20001":
                        quadratic[name]["sampled_max_abs_C_20001"],
                    "sampled_C_within_limit":
                        quadratic[name]["sampled_C_within_limit"],
                    "exact_endpoint": quadratic[name]["exact_endpoint"],
                }
                for name in sorted(quadratic)},
            "straight_spline": {
                "bc_type": straight["bc_type"],
                "stations_m": straight["stations_m"],
                "curvature_1_per_m": straight["curvature_1_per_m"],
                "slope_1_per_m2": straight["slope_1_per_m2"],
                "exact_endpoint": straight["exact_endpoint"],
            },
        },
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }
    target = frozen_dir / "frozen_manifest.json"
    target.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "FROZEN",
        "files": {name: record["frozen_sha256"]
                  for name, record in records.items()},
        "planner_config_is_default": True,
        "requests": len(request_records),
        "request_fingerprints": {
            row["name"]: row["request_sha256"] for row in request_records},
        "manifest_sha256": sha256(target),
        "manifest_bytes": target.stat().st_size,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()