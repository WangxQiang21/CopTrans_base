#!/usr/bin/env python3
"""Step 5 offline probe: versioned v3 numerics certificate and v2 replay.

No ROS process starts.  The historical v2 model is rebuilt from its stored
JSON and compared byte for byte; the 600003/600004 and synthetic 50 m/100 m
constructions are solved with the frozen Step 4 planner policy and their v3
production certificates are reported segment by segment.  Each solved profile
is also rebuilt with the historical v2 planner on the same knots, so the
v2 -> v3 boundary is measured on identical geometry.  Version tampering is
rejected through both the model reader and the independent audit.

This is analysis/evidence code, not planner source.  Nothing in the
certificate is trusted from the result: the printed fields come from the
production objects and are cross-checked in the accompanying summary.
"""
import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
UTURN = ROOT / "evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z"
V2_ENVELOPE = (
    ROOT / "evaluation/virtual_formation_a/G5/20261116T112522Z/"
    "final_main_free_5/envelopes/envelope_00600000.json")
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(UTURN))

from fwis_v3_optimizer.formation import FormationPose  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a import bounds  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a.audit import (  # noqa: E402
    audit_dynamics)
from fwis_v3_optimizer.formation_continuous_a.collision import (  # noqa: E402
    load_map)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    PlannerConfig,
    VirtualAContractError,
    request_fingerprint)
from fwis_v3_optimizer.formation_continuous_a.geometry import (  # noqa: E402
    C2CurvatureProfile,
    VirtualAPath)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (  # noqa: E402
    make_initial_guesses,
    solve_open_path_guided)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (  # noqa: E402
    VirtualABackendSettings,
    VirtualAInitialContext,
    request_from_snapshot,
    solve)
from fwis_v3_optimizer.formation_continuous_a.trajectory import (  # noqa: E402
    VirtualATimedPlan)
from fwis_v3_optimizer.formation_request import (  # noqa: E402
    validate_formation_request)
from repro_u_turn import PLANS, build_message, jsonable  # noqa: E402

REAL_PLAN_START = PLANS[600003]["start"]


def polyline_length(points):
    return float(sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])))


def certificate_summary(path):
    budget = path.integration_budget
    version = getattr(path, "numerics_version", None)
    summary = {
        "kind": getattr(path, "kind", None),
        "length_m": float(path.length_m),
        "numerics_version": version,
        "position_error_bound_m": float(path.position_error_bound_m),
        "angle_error_bound_rad": float(path.angle_error_bound_rad),
        "position_error_bound_within_1e_7": bool(
            path.position_error_bound_m <= bounds.POSITION_BUDGET_M),
        "integration_budget": jsonable(budget),
    }
    if version == bounds.NUMERICS_V3:
        segments = []
        for segment in budget["segments"]:
            segments.append({
                "segment_index": segment["segment_index"],
                "s0": segment["s0"], "s1": segment["s1"],
                "h": segment["h"],
                "subintervals": segment["subintervals"],
                "full_panel_count": segment["full_panel_count"],
                "maximum_panel_m4_upper_m":
                    segment["maximum_panel_m4_upper_m"],
                "quadrature_remainder_m":
                    segment["quadrature_remainder_m"],
                "evaluation_guard_m": segment["evaluation_guard_m"],
                "angle_guard_m": segment["angle_guard_m"],
                "per_coordinate_total_m":
                    segment["per_coordinate_total_m"],
            })
        summary["v3_certificate"] = {
            "m4_method": budget["m4_method"],
            "quadrature_method": budget["quadrature_method"],
            "outward_rounding_method": budget["outward_rounding_method"],
            "per_coordinate_quadrature_m":
                budget["per_coordinate_quadrature_m"],
            "per_coordinate_evaluation_m":
                budget["per_coordinate_evaluation_m"],
            "per_coordinate_angle_m": budget["per_coordinate_angle_m"],
            "per_coordinate_total_m": budget["per_coordinate_total_m"],
            "euclidean_scale": budget["euclidean_scale"],
            "total_budget_m": budget["total_budget_m"],
            "total_panels": budget["total_panels"],
            "panels_per_segment": budget["panels_per_segment"],
            "initial_panels_per_segment":
                budget["initial_panels_per_segment"],
            "refinement_steps": budget["refinement_steps"],
            "refined_segments": budget["refined_segments"],
            "floating_point_guard_m": budget["floating_point_guard_m"],
            "evaluation_guard_components":
                budget["evaluation_guard_components"],
            "segments": segments,
        }
    return summary


def report_summary(report):
    if report is None:
        return None
    return {
        "passed": bool(getattr(report, "passed", False)),
        "reason": str(getattr(report, "reason", "") or ""),
        "detail": str(getattr(report, "detail", "") or ""),
        "scope": str(getattr(report, "scope", "") or ""),
        "location": jsonable(dict(getattr(report, "location", {}) or {})),
        "metrics": jsonable(getattr(report, "metrics", {}) or {}),
    }


def rebuild_certificate(knots, start_qf, model, version):
    """Rebuild the same raw knots with an explicitly chosen version."""
    profile = C2CurvatureProfile.from_knots(
        knots, numerics_version=version)
    try:
        rebuilt = VirtualAPath(
            start_qf, profile, model, numerics_version=version)
    except VirtualAContractError as exc:
        return {
            "requested_version": version,
            "status": exc.reason,
            "detail": exc.detail,
            "location": jsonable(dict(getattr(exc, "location", {}) or {})),
            "same_knots": True,
        }
    return {
        "requested_version": version,
        "status": "CONFIRMED",
        "same_knots": True,
        "certificate": certificate_summary(rebuilt),
    }


def replay_v2_model():
    envelope = json.loads(V2_ENVELOPE.read_text())
    payload = envelope["model_json"]
    plan = VirtualATimedPlan.from_model_json(payload)
    rebuilt = plan.to_model_json()
    document = json.loads(payload)
    return {
        "kind": "v2_replay",
        "source": str(V2_ENVELOPE),
        "plan_id": envelope["plan_id"],
        "stored_model_sha256": envelope["model_sha256"],
        "rebuilt_model_sha256": plan.model_sha256(),
        "byte_identical": rebuilt == payload,
        "stored_model_length": len(payload),
        "rebuilt_model_length": len(rebuilt),
        "declared_version": document["numerics"]["version"],
        "profile_version": document["path"]["profile"]["numerics_version"],
        "path_numerics_version": plan.path.numerics_version,
        "integration_budget_version":
            plan.path.integration_budget.get("numerics_version"),
        "integration_budget_matches":
            plan.path.integration_budget == document["path"][
                "integration_budget"],
        "position_error_bound_m": plan.path.position_error_bound_m,
        "panels_per_segment":
            plan.path.integration_budget["panels_per_segment"],
        "v3_rebuild_of_same_knots": rebuild_certificate(
            plan.path.profile.knots, plan.path.start_qf, plan.path.model,
            bounds.NUMERICS_V3),
    }


def declared_version(document, block):
    if block == "profile":
        return document["path"]["profile"]["numerics_version"]
    return document["numerics"]["version"]


def tamper_checks(plan, request, planning_context):
    document = json.loads(plan.to_model_json())

    def mutate(core, block, value):
        if block == "profile":
            core["path"]["profile"]["numerics_version"] = value
        else:
            core[block]["version"] = value
        return core

    results = {}
    for name, block in (("numerics", "numerics"), ("profile", "profile")):
        current = declared_version(document, block)
        other = bounds.NUMERICS_V2 if current == bounds.NUMERICS_V3 \
            else bounds.NUMERICS_V3
        cases = {}
        for label, value in (
                ("unknown", "virtual_rigid_a.numerics.g1.v9"),
                ("other_supported", other)):
            mutated = copy.deepcopy(document)
            mutate(mutated, block, value)
            try:
                VirtualATimedPlan.from_model_json(json.dumps(mutated))
                reason = "ACCEPTED"
            except VirtualAContractError as exc:
                reason = exc.reason
            cases[label] = {"value": value, "reason": reason}
        results[block] = cases
    tampered = copy.deepcopy(plan)
    tampered._model_core["numerics"]["version"] = "other"
    results["audit_numerics"] = report_summary(audit_dynamics(
        request, tampered, expected_planning_context=planning_context))
    tampered = copy.deepcopy(plan)
    tampered._model_core["path"]["profile"]["numerics_version"] = "other"
    results["audit_profile"] = report_summary(audit_dynamics(
        request, tampered, expected_planning_context=planning_context))
    return results


def run_real(args, plan_id):
    spec = PLANS[plan_id]
    grid, metadata = load_map(args.map_yaml)
    message = build_message(plan_id, spec["start"], spec["goal"])
    validation = validate_formation_request(message)
    if not validation.valid:
        raise SystemExit("plan {} request invalid: {} {} {}".format(
            plan_id, validation.reason, validation.detail,
            validation.field))
    settings = VirtualABackendSettings(
        planner_config=PlannerConfig(
            guidance_mode=args.guidance_mode,
            intervals=args.intervals,
            max_initials=args.max_initials,
            max_seconds=args.max_seconds,
            max_iterations=args.max_iterations,
            solver=args.solver),
        goal_policy="free",
        frame_id="map")
    context = VirtualAInitialContext(
        source="rviz_reset",
        context_id="offline-numerics-{}-1".format(plan_id),
        qf=FormationPose(*spec["start"]))
    map_request = request_from_snapshot(
        validation.snapshot, grid, settings, context)
    started = time.monotonic()
    result = solve(map_request)
    wall = time.monotonic() - started
    search = dict(getattr(result, "search", {}) or {})
    guide = None
    rounds = tuple(dict(search.get("corridor") or {}).get("rounds", ()) or ())
    if rounds:
        guide = rounds[-1].get("guide")
    guide_length = polyline_length(guide.get("points") or ()) \
        if isinstance(guide, dict) else None
    path = getattr(result, "path", None)
    plan = getattr(result, "plan", None)
    document = {
        "kind": "real",
        "plan_id": plan_id,
        "status": str(getattr(result, "status", "") or ""),
        "reason": str(getattr(result, "reason", "") or ""),
        "detail": str(getattr(result, "detail", "") or ""),
        "first_failure_layer": str(
            getattr(result, "first_failure_layer", "") or ""),
        "executable": bool(getattr(result, "executable", False)),
        "wall_time_s": float(getattr(result, "wall_time_s", wall)),
        "request_sha256": getattr(result, "request_sha256", ""),
        "guide_length_m": guide_length,
        "geometry_audit": report_summary(
            getattr(result, "geometry_audit", None)),
        "dynamic_audit": report_summary(
            getattr(result, "dynamic_audit", None)),
        "collision_audit": report_summary(
            getattr(result, "collision_audit", None)),
        "attempts": [
            {
                "attempt_index": getattr(item, "attempt_index", None),
                "status": getattr(item, "status", None),
                "solver_status": getattr(item, "solver_status", None),
                "S_solution_m": getattr(item, "s_solution_m", None),
                "rejection_reason": getattr(item, "rejection_reason", None),
                "detail": getattr(item, "detail", None),
            }
            for item in (getattr(result, "attempts", ()) or ())
        ],
        "reproduction": {
            "map_yaml": str(args.map_yaml),
            "map_metadata": jsonable(metadata),
            "grid_sha256": map_request.grid_sha256,
            "request_sha256": map_request.request_sha256,
            "planner_config": jsonable(
                map_request.planner_config.as_dict()),
            "start": spec["start"],
            "goal": spec["goal"],
        },
    }
    if path is not None:
        document["path"] = certificate_summary(path)
        document["v2_rebuild_of_same_knots"] = rebuild_certificate(
            path.profile.knots, path.start_qf, path.model,
            bounds.NUMERICS_V2)
    if plan is not None:
        payload = plan.to_model_json()
        document["model"] = {
            "sha256": plan.model_sha256(),
            "length": len(payload),
            "numerics_version": plan.path.numerics_version,
            "declared_values": {
                "numerics": json.loads(payload)["numerics"]["version"],
                "profile":
                    json.loads(payload)["path"]["profile"][
                        "numerics_version"],
                "integration_budget": json.loads(payload)["path"][
                    "integration_budget"]["numerics_version"],
            },
            "roundtrip_byte_identical":
                VirtualATimedPlan.from_model_json(payload).to_model_json()
                == payload,
            "planning_context_sha256": plan.planning_context_sha256(),
        }
        document["version_tamper"] = tamper_checks(
            plan, map_request.request, map_request.context())
    return document


def run_synthetic(args, length_m, plan_id):
    start = REAL_PLAN_START
    goal = (start[0] + float(length_m), start[1], 0.0)
    grid, metadata = load_map(args.map_yaml)
    message = build_message(plan_id, start, goal)
    validation = validate_formation_request(message)
    if not validation.valid:
        raise SystemExit("synthetic {} invalid: {} {} {}".format(
            length_m, validation.reason, validation.detail,
            validation.field))
    config = PlannerConfig(
        guidance_mode="legacy_route",
        intervals=args.intervals,
        max_initials=args.max_initials,
        max_seconds=args.max_seconds,
        max_iterations=args.max_iterations,
        solver=args.solver)
    settings = VirtualABackendSettings(
        planner_config=config, goal_policy="free", frame_id="map")
    context = VirtualAInitialContext(
        source="rviz_reset",
        context_id="offline-numerics-synthetic-{}-1".format(plan_id),
        qf=FormationPose(*start))
    map_request = request_from_snapshot(
        validation.snapshot, grid, settings, context)
    request = map_request.request
    started = time.monotonic()
    result = solve_open_path_guided(request, config)
    wall = time.monotonic() - started
    path = getattr(result, "path", None)
    document = {
        "kind": "synthetic_straight",
        "plan_id": plan_id,
        "length_m": float(length_m),
        "status": str(getattr(result, "status", "") or ""),
        "reason": str(getattr(result, "reason", "") or ""),
        "detail": str(getattr(result, "detail", "") or ""),
        "wall_time_s": wall,
        "request_sha256": request_fingerprint(request),
        "initial_S_values_m": [
            float(guess.S) for guess in make_initial_guesses(request, config)],
        "attempts": [
            {
                "attempt_index": getattr(item, "attempt_index", None),
                "status": getattr(item, "status", None),
                "S_solution_m": getattr(item, "s_solution_m", None),
                "rejection_reason": getattr(item, "rejection_reason", None),
                "detail": getattr(item, "detail", None),
            }
            for item in (getattr(result, "attempts", ()) or ())
        ],
    }
    if path is not None:
        document["path"] = certificate_summary(path)
        document["v2_rebuild_of_same_knots"] = rebuild_certificate(
            path.profile.knots, path.start_qf, path.model,
            bounds.NUMERICS_V2)
    return document


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--guidance-mode", default="corridor")
    parser.add_argument("--intervals", type=int, default=8)
    parser.add_argument("--max-initials", type=int, default=6)
    parser.add_argument("--max-seconds", type=float, default=30.0)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--solver", default="ipopt")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    documents = [replay_v2_model()]
    documents.extend(run_real(args, plan_id)
                     for plan_id in (600003, 600004))
    documents.append(run_synthetic(args, 50.0, 900001))
    documents.append(run_synthetic(args, 100.0, 900002))
    Path(args.output).write_text(
        json.dumps(documents, indent=2, sort_keys=True) + "\n")
    for document in documents:
        label = document.get("plan_id", "v2_replay")
        if document["kind"] == "v2_replay":
            print("v2_replay byte_identical={} sha={} budget_version={}".format(
                document["byte_identical"],
                document["rebuilt_model_sha256"],
                document["integration_budget_version"]))
            v3 = document["v3_rebuild_of_same_knots"]
            print("  v3_same_knots status={} bound={} panels={}".format(
                v3["status"],
                (v3.get("certificate") or {}).get(
                    "position_error_bound_m"),
                (v3.get("certificate") or {}).get(
                    "integration_budget", {}).get(
                        "total_panels", v3.get("location", {}).get(
                            "total_panels"))))
            continue
        print("plan={} kind={} status={} reason={} wall={:.3f}s".format(
            label, document["kind"], document["status"],
            document["reason"], document["wall_time_s"]))
        path = document.get("path")
        if path is not None:
            cert = path.get("v3_certificate") or {}
            print("  path length={:.6f} version={} bound={:.3e} "
                  "angle={:.3e} panels={} refine={} eval={:.3e}".format(
                      path["length_m"], path["numerics_version"],
                      path["position_error_bound_m"],
                      path["angle_error_bound_rad"],
                      cert.get("total_panels"),
                      cert.get("refinement_steps"),
                      cert.get("per_coordinate_evaluation_m")))
            v2 = document["v2_rebuild_of_same_knots"]
            print("  v2_same_knots status={} panels={}".format(
                v2["status"],
                (v2.get("certificate") or {}).get(
                    "integration_budget", {}).get(
                        "total_panels", v2.get("location", {}).get(
                            "total_panels"))))
        audits = [document.get(name) for name in (
            "geometry_audit", "dynamic_audit", "collision_audit")]
        if any(audit is not None for audit in audits):
            print("  audits passed={} reasons={}".format(
                [audit["passed"] if audit else None for audit in audits],
                [audit["reason"] if audit else None for audit in audits]))
        model = document.get("model")
        if model:
            print("  model sha={} len={} declared={}".format(
                model["sha256"], model["length"],
                model["declared_values"]))


if __name__ == "__main__":
    main()