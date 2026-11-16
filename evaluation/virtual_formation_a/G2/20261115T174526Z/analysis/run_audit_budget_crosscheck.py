"""Numerically cross-check the independent audit Simpson budget.

The production audit rebuilds curvature pieces from raw knots and integrates
cos/sin(psi(s)) with composite Simpson plus a certified ``H^5 M4 / 2880``
remainder.  This script audits a legal multi-segment cubic curvature profile
with a nonzero fourth derivative and compares the audit's independently
reconstructed endpoint against SciPy adaptive quadrature.  It also records a
refinement case and two resource-limited BOUND_UNCONFIRMED negatives.

This is evidence only; it never feeds the profile or quadrature result to the
production solver.
"""
import datetime
import json
import math
from pathlib import Path

from scipy.integrate import quad

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a.audit import audit_geometry
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GeometryAuditConfig,
    GoalPolicy,
    InitialAssumption,
    VirtualARequest,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures" / "virtual_a"
REQUESTS = json.loads(
    (FIXTURE_DIR / "open_requests.json").read_text(encoding="utf-8"))
BASE = REQUESTS["base"]


def model_from_fixture():
    return FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in BASE["model"]["offsets"]])


def shapes_from_fixture():
    return {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in BASE["model"]["vehicle_shapes"]}


def request_from_fixture(name, goal):
    record = REQUESTS["requests"][name]
    return VirtualARequest(
        plan_id=record["plan_id"],
        formation_id=record["formation_id"],
        model=model_from_fixture(),
        vehicle_shapes=shapes_from_fixture(),
        start_qf=FormationPose(**BASE["start_qf"]),
        start_headings={
            int(key): float(value)
            for key, value in BASE["start_headings"].items()},
        goal_qf=goal,
        initial=InitialAssumption(**BASE["initial"]),
        goal_policy=GoalPolicy(record["goal_policy"]),
        limits=FormationTimingLimits(**BASE["limits"]))


def nonzero_g_profile():
    b, c = 0.012, -0.0015
    stations = (0.0, 1.1, 2.4, 3.9, 5.2)
    return C2CurvatureProfile.from_knots([
        ControlKnot(s, b * s * s + c * s ** 3,
                    2.0 * b * s + 3.0 * c * s * s)
        for s in stations])


def high_precision_endpoint(path):
    stations = [0.0] + [
        piece["s0"] for piece in path.profile.as_dict()["pieces"][1:]]
    stations.append(path.length_m)
    psi = path.profile.psi_at
    kwargs = dict(epsabs=1e-13, epsrel=1e-13, limit=400)
    x_value, x_error, x_info = quad(
        lambda s: math.cos(psi(s)), 0.0, path.length_m,
        points=stations, full_output=1, **kwargs)
    y_value, y_error, y_info = quad(
        lambda s: math.sin(psi(s)), 0.0, path.length_m,
        points=stations, full_output=1, **kwargs)
    return {
        "x": x_value, "y": y_value, "psi": psi(path.length_m),
        "quad_x_abserr": x_error, "quad_y_abserr": y_error,
        "quad_x_message": str(x_info.get("message", "")),
        "quad_y_message": str(y_info.get("message", "")),
        "quad_x_neval": int(x_info.get("neval", 0)),
        "quad_y_neval": int(y_info.get("neval", 0)),
    }


def audit_record(report):
    c2 = report.c2 or {}
    return {
        "passed": report.passed,
        "reason": report.reason,
        "location": report.location,
        "integration": report.integration,
        "whole_segment_count": len(report.whole_segment_bounds),
        "c2_max_abs_g": c2.get("max_abs_g"),
    }


def main():
    profile = nonzero_g_profile()
    path = VirtualAPath(
        FormationPose(0.0, 0.0, 0.0), profile, model_from_fixture())
    endpoint = path.endpoint()
    request = request_from_fixture(
        "STRAIGHT_LINE",
        FormationPose(endpoint.x, endpoint.y, endpoint.psi))
    path.request_sha256 = request_fingerprint(request)

    exact = high_precision_endpoint(path)
    default_report = audit_geometry(request, path)
    coarse_report = audit_geometry(
        request, path, GeometryAuditConfig(audit_panel_width_m=1.0))
    resource_report = audit_geometry(
        request, path, GeometryAuditConfig(
            audit_max_total_subintervals=4))
    unreachable_report = audit_geometry(
        request, path, GeometryAuditConfig(
            audit_position_budget_m=1e-15,
            audit_max_subintervals_per_segment=64,
            audit_max_total_subintervals=64))

    integration = default_report.integration
    independent = integration["independent_endpoint"]
    budget = integration["audit_integration"]
    independent_difference = {
        "x": independent["x"] - exact["x"],
        "y": independent["y"] - exact["y"],
        "euclidean": math.hypot(
            independent["x"] - exact["x"],
            independent["y"] - exact["y"]),
    }
    returned_difference = {
        "x": endpoint.x - independent["x"],
        "y": endpoint.y - independent["y"],
        "euclidean": math.hypot(
            endpoint.x - independent["x"],
            endpoint.y - independent["y"]),
    }
    checks = {
        "default_audit_passes": default_report.passed is True,
        "c2_third_derivative_nonzero": default_report.c2["max_abs_g"] > 0.0,
        "four_segments": len(default_report.whole_segment_bounds) == 4,
        "audit_budget_confirmed": budget["confirmed"] is True,
        "audit_threshold_strictly_below_g1": budget[
            "fixed_threshold_m"] < 1e-7,
        "audit_total_within_threshold": budget["total_budget_m"] <= budget[
            "fixed_threshold_m"],
        "independent_vs_quad_within_audit_budget":
            independent_difference["euclidean"] <= budget["total_budget_m"]
            + exact["quad_x_abserr"] + exact["quad_y_abserr"],
        "returned_vs_independent_within_combined_budget":
            returned_difference["euclidean"] <= integration[
                "combined_position_budget_m"],
        "coarse_grid_refined": coarse_report.passed is True and coarse_report
            .integration["audit_integration"]["refinement_steps"] > 0,
        "resource_limit_unconfirmed": (
            resource_report.passed is False
            and resource_report.reason == "BOUND_UNCONFIRMED"
            and resource_report.location.get("constraint") ==
            "AUDIT_INTEGRATION_TOTAL_LIMIT"),
        "unreachable_budget_unconfirmed": (
            unreachable_report.passed is False
            and unreachable_report.reason == "BOUND_UNCONFIRMED"),
    }
    document = {
        "schema": "virtual_rigid_a.g2.audit_budget_crosscheck.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "profile": {
            "definition": "k(s)=0.012*s^2-0.0015*s^3 on stations "
                          "0,1.1,2.4,3.9,5.2 m",
            "length_m": path.length_m,
            "fourth_derivative_note": (
                "third curvature derivative is the nonzero constant 6*c; "
                "the position integrand has nonzero fourth derivative"),
            "knots": [
                {"s": knot.s, "kappa": knot.value,
                 "eta": knot.derivative}
                for knot in profile.knots],
        },
        "formula": {
            "method": budget["method"],
            "remainder": budget["formula"],
            "m4_definition": "G + 4*K*U + 3*E^2 + 6*K^2*E + K^4",
            "panel_width": "2*h/subintervals",
            "floating_point_assumption": budget[
                "floating_point_assumption"],
        },
        "default_audit": audit_record(default_report),
        "coarse_refinement_audit": audit_record(coarse_report),
        "resource_limited_audit": audit_record(resource_report),
        "unreachable_budget_audit": audit_record(unreachable_report),
        "high_precision_quadrature": exact,
        "comparison": {
            "independent_minus_quad": independent_difference,
            "returned_path_minus_independent": returned_difference,
            "combined_position_budget_m": integration[
                "combined_position_budget_m"],
            "audit_total_budget_m": budget["total_budget_m"],
            "quad_abserr_sum_m": exact["quad_x_abserr"] + exact[
                "quad_y_abserr"],
        },
        "checks": checks,
        "all_checks_pass": all(checks.values()),
        "nonclaims": [
            "numerical comparison, not a proof of libm/trig rounding",
            "the audit budget covers Simpson truncation plus a documented "
            "floating-point/angle guard",
            "this profile is an audit fixture, never a solver seed",
        ],
    }
    target = RUN / "analysis" / "audit_budget_crosscheck.json"
    target.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "PASS" if all(checks.values()) else "FAIL",
        "run_id": RUN.name,
        "target": str(target.relative_to(REPO_ROOT)),
        "target_sha256": __import__("hashlib").sha256(
            target.read_bytes()).hexdigest(),
        "checks": checks,
        "audit_total_budget_m": budget["total_budget_m"],
        "audit_fixed_threshold_m": budget["fixed_threshold_m"],
        "independent_minus_quad_m": independent_difference,
        "returned_minus_independent_m": returned_difference,
        "coarse_refinement_steps": coarse_report.integration[
            "audit_integration"]["refinement_steps"],
        "resource_reason": resource_report.reason,
        "resource_constraint": resource_report.location.get("constraint"),
        "unreachable_reason": unreachable_report.reason,
    }, indent=2, sort_keys=True))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    import sys
    sys.exit(main())