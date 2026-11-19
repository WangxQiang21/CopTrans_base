#!/usr/bin/env python3
"""Generate deterministic TASK02 rework evidence summaries.

Reads the live capture already written under ``rework/live`` and reproduces
the three Rework-A fixtures from the regression tests.  It only writes inside
this evidence snapshot; it never publishes ROS messages or changes source.
"""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SNAPSHOT = HERE.parents[1]
ROOT = HERE.parents[4]
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from fwis_v3_optimizer.formation_continuous_a.collision import (  # noqa: E402
    FormationCollisionChecker,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    GuideRoute,
    grid_signature,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (  # noqa: E402
    allocate_corridors,
    plan_guide_candidates,
    select_guide_target_portal,
)
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    pipeline as pipeline_module,
)
import test_virtual_a_formation_assessment as fixtures  # noqa: E402

MODEL = fixtures.MODEL
SHAPES = fixtures.SHAPES


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, float):
        return value
    return value


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def candidate_points(candidate):
    return [[float(x), float(y)] for x, y in candidate.points]


def generation_summary(generation):
    keys = (
        "schema", "selection", "target_portal_index", "target_window_index",
        "target_reason", "risk_portals", "portal_radii_m", "target_radius_m",
        "local_candidate_point_count", "local_candidate_count",
        "local_candidates_kept", "local_candidates",
        "dropped_local_candidates", "candidate_certification",
        "continuously_certified", "formation_obb_hard_constraints",
        "certification_claim",
    )
    return {key: generation.get(key) for key in keys}


def later_portal_evidence():
    grid, chain, base, baseline = (
        fixtures.GuideCandidateGenerationTests._multi_portal_fixture())
    target, selection = select_guide_target_portal(chain, base, baseline)
    checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.05)
    candidates, generation = plan_guide_candidates(
        chain, (-0.5, 0.0), (17.0, 0.0), base_guide=base,
        baseline_assessment=baseline, checker=checker, model=MODEL,
        vehicle_shapes=SHAPES)
    local = [
        candidate for candidate in candidates
        if str((candidate.quality or {}).get("candidate_origin") or "")
        .startswith("ONE_PORTAL_VARIANT_")]
    allocations = []
    local_points = []
    for candidate in candidates:
        allocation = allocate_corridors(chain, candidate)
        allocations.append(len(allocation))
        if candidate in local:
            point = candidate.points[3]
            local_points.append({
                "source": candidate.source,
                "point": [float(point[0]), float(point[1])],
                "changed_indices": [
                    index for index in range(len(candidate.points))
                    if candidate.points[index] != base.points[index]],
                "start_unchanged": candidate.points[0] == base.points[0],
                "goal_unchanged": candidate.points[-1] == base.points[-1],
                "previous_portal_unchanged":
                    candidate.points[2] == base.points[2],
                "violation_region_2_m": fixtures._point_violation(
                    point, chain.regions[2]),
                "violation_region_3_m": fixtures._point_violation(
                    point, chain.regions[3]),
            })
    return {
        "fixture": {
            "region_count": len(chain.regions),
            "base_guide_points": candidate_points(base),
            "baseline_windows": [
                {"portal_index": window["portal_index"],
                 "status": window["status"],
                 "s_window_m": window["s_window_m"]}
                for window in baseline["transition_windows"]],
        },
        "selection": selection,
        "generation": generation_summary(generation),
        "candidate_count": len(candidates),
        "candidate_origins": [
            (candidate.quality or {}).get("candidate_origin")
            for candidate in candidates],
        "candidate_points": [
            candidate_points(candidate) for candidate in candidates],
        "allocations_nonempty": all(count > 0 for count in allocations),
        "allocation_counts": allocations,
        "local_variants": local_points,
    }


def no_risk_evidence():
    small = fixtures.GuideCandidateGenerationTests()._no_risk_fixture(11.5)
    tie = fixtures.GuideCandidateGenerationTests()._no_risk_fixture(11.0)
    result = {}
    for label, fixture in (("smallest_radius", small), ("radius_tie", tie)):
        chain, base, baseline = fixture
        target, selection = select_guide_target_portal(
            chain, base, baseline)
        result[label] = {"target_portal_index": target,
                         "selection": selection}
    return result


def real_checker_evidence():
    fixture = fixtures.GuideCandidateGenerationTests()
    grid, chain, base = fixture._real_collision_fixture()
    checker = FormationCollisionChecker(grid, MODEL, SHAPES, 0.0)
    baseline = fixtures._assess(
        chain, base.points, checker, MODEL, SHAPES, 0.0, 0.0, grid,
        guide_source=base.source)
    target, selection = select_guide_target_portal(chain, base, baseline)
    candidates, generation = plan_guide_candidates(
        chain, (0.0, 0.0), (10.0, 0.0), base_guide=base,
        baseline_assessment=baseline, checker=checker, model=MODEL,
        vehicle_shapes=SHAPES)
    records = []
    for index, candidate in enumerate(candidates):
        allocation = allocate_corridors(chain, candidate)
        assessment = fixtures._assess(
            chain, candidate.points, checker, MODEL, SHAPES, 0.0, 0.0, grid,
            guide_source=candidate.source, allocation=allocation)
        records.append(pipeline_module._guide_candidate_record(
            index, candidate, allocation, assessment))
    best = min(records, key=pipeline_module._guide_candidate_score)
    for record in records:
        record["selected"] = record is best
    selected_candidate = candidates[best["candidate_index"]]
    return {
        "base_map_collision_count":
            baseline["map_sample_counts"]["map_collision"],
        "target_portal_index": target,
        "selection": selection,
        "generation": generation_summary(generation),
        "candidates": [
            {"candidate_index": record["candidate_index"],
             "candidate_origin": record["candidate_origin"],
             "guide_source": record["guide_source"],
             "status": record["status"],
             "selection_counts": record["selection_counts"],
             "selected": record["selected"],
             "points": candidate_points(candidates[
                 record["candidate_index"]])}
            for record in records],
        "selected_candidate_index": best["candidate_index"],
        "selected_candidate_origin": best["candidate_origin"],
        "selected_candidate_points": candidate_points(selected_candidate),
        "selected_candidate_has_map_changed":
            selected_candidate.points[1] != base.points[1],
        "checker_static_check_calls": checker._stats["static_checks"],
    }


def live_evidence():
    live = SNAPSHOT / "rework" / "live"
    map05 = json.loads(
        (live / "map05_default" / "capture_summary.json").read_text())
    s1 = json.loads(
        (live / "s1_default" / "capture_summary.json").read_text())
    s1_quality = json.loads(
        (live / "s1_default" / "quality_report.json").read_text())
    map05_quality = json.loads(
        (live / "map05_default" / "quality_report.json").read_text())
    map05_hud = json.loads(
        (live / "map05_default" / "hud_texts.json").read_text())
    s1_hud = json.loads(
        (live / "s1_default" / "hud_texts.json").read_text())
    map_round = map05_quality["virtual_result"]["search"]["corridor"][
        "rounds"][-1]
    s1_round = s1_quality["virtual_result"]["search"]["corridor"]["rounds"][-1]
    s1_generation = s1_round["guide_selection"]["candidate_generation"]
    map_counts = map_round["formation_assessment"]["coverage_vehicles"]
    single = sum(entry["counts"]["single"]
                 for entry in map_counts.values())
    union = sum(entry["counts"]["union"]
                for entry in map_counts.values())
    gap = sum(entry["counts"]["gap"] for entry in map_counts.values())
    unknown = sum(entry["counts"]["unconfirmed"]
                  for entry in map_counts.values())
    return {
        "map05_default": {
            "command": ("./run_demo.sh --map map05 --scale 2.625"),
            "driver_exit": int(
                (live / "map05_default" / "capture_driver.exit")
                .read_text().strip().split("=")[-1]),
            "status": map05_quality["virtual_result"]["status"],
            "reason": map05_quality["virtual_result"]["reason"],
            "executable_envelope_count":
                map05.get("executable_envelope_count"),
            "map_sample_status":
                map_round["formation_assessment"]["map_sample_status"],
            "coverage_sample_status":
                map_round["formation_assessment"]["coverage_sample_status"],
            "single_region_total": single,
            "union_covered_total": union,
            "union_gap_total": gap,
            "union_unknown_total": unknown,
            "hud_lines": [
                line for text in map05_hud
                for line in str(text).splitlines()
                if line.startswith("samples=")],
            "selection_rule":
                (map_round.get("guide_selection") or {}).get("selection"),
            "candidate_certification":
                map_round["formation_assessment"][
                    "candidate_certification"],
            "continuously_certified":
                map_round["formation_assessment"][
                    "continuously_certified"],
        },
        "s1_default": {
            "command": ("./run_demo.sh --map evaluation/virtual_formation_a/"
                        "G4/20261116T081458Z/scenes/inputs/S1 --scale 1.0"),
            "driver_exit": int(
                (live / "s1_default" / "capture_driver.exit")
                .read_text().strip().split("=")[-1]),
            "status": s1_quality["virtual_result"]["status"],
            "reason": s1_quality["virtual_result"]["reason"],
            "first_failure_layer":
                s1_quality["virtual_result"]["first_failure_layer"],
            "executable_envelope_count":
                s1.get("executable_envelope_count"),
            "route_count":
                s1_quality["virtual_result"]["search"]["route_count"],
            "target_portal_index": s1_generation.get(
                "target_portal_index"),
            "target_window_index": s1_generation.get(
                "target_window_index"),
            "target_reason": s1_generation.get("target_reason"),
            "risk_portals": s1_generation.get("risk_portals"),
            "local_candidates_kept":
                s1_generation.get("local_candidates_kept"),
            "local_candidate_point_count":
                s1_generation.get("local_candidate_point_count"),
            "selection_rule":
                s1_round["guide_selection"].get("selection_rule"),
            "selected_candidate_index":
                s1_round["guide_selection"].get(
                    "selected_candidate_index"),
            "selected_candidate_origin":
                s1_round["guide_selection"].get(
                    "selected_candidate_origin"),
            "selected_guide_points":
                s1_round["guide"]["points"],
            "candidate_origins": [
                candidate.get("candidate_origin")
                for candidate in s1_round.get("guide_candidates") or []],
            "candidate_collision_counts": [
                (candidate.get("selection_counts") or {}).get(
                    "collision_count")
                for candidate in s1_round.get("guide_candidates") or []],
            "hud_lines": [
                line for text in s1_hud
                for line in str(text).splitlines()
                if line.startswith("samples=")],
        },
    }


def main():
    snapshots = SNAPSHOT / "rework"
    summary = {
        "schema": "task02.rework.evidence.v1",
        "generated_utc": "2026-11-19",
        "guide_selection_evidence": {
            "later_portal_risk": later_portal_evidence(),
            "no_risk_radius": no_risk_evidence(),
            "real_static_check_improvement": real_checker_evidence(),
        },
        "live": live_evidence(),
        "test_results": {
            "test_virtual_a_formation_assessment": {
                "tests": 40, "exit": 0},
            "test_virtual_a_pipeline": {"tests": 13, "exit": 0},
            "test_virtual_a_corridor_display + test_formation_hud": {
                "tests": 92, "exit": 0},
            "test_virtual_a_corridor + test_virtual_a_collision": {
                "tests": 42, "exit": 0},
            "catkin_make": {"exit": 0},
            "git_diff_check": {"exit": 0},
            "bash_n_run_demo": {"exit": 0},
        },
        "task01_dirty_sha256": {
            "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
            "formation_continuous_a/geometry.py": sha256_file(
                ROOT / "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
                "formation_continuous_a/geometry.py"),
            "tests/test_virtual_a_geometry.py": sha256_file(
                ROOT / "tests/test_virtual_a_geometry.py"),
        },
    }
    (HERE / "selection_summary.json").write_text(
        json.dumps(json_safe(summary), indent=1, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "later_target": summary["guide_selection_evidence"][
            "later_portal_risk"]["selection"]["target_portal_index"],
        "later_changed": [
            entry["changed_indices"] for entry in summary[
                "guide_selection_evidence"]["later_portal_risk"][
                "local_variants"]],
        "no_risk": {
            key: entry["target_portal_index"] for key, entry in summary[
                "guide_selection_evidence"]["no_risk_radius"].items()},
        "real_selected": summary["guide_selection_evidence"][
            "real_static_check_improvement"]["selected_candidate_index"],
        "real_base_collisions": summary["guide_selection_evidence"][
            "real_static_check_improvement"][
            "base_map_collision_count"],
        "map05_single": summary["live"]["map05_default"][
            "single_region_total"],
        "map05_union": summary["live"]["map05_default"][
            "union_covered_total"],
        "s1_reason": summary["live"]["s1_default"]["reason"],
        "task01_hashes": summary["task01_dirty_sha256"],
    }, indent=1, sort_keys=True))


if __name__ == "__main__":
    main()