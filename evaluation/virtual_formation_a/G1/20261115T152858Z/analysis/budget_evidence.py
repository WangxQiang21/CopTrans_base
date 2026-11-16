"""Emit reproducible numerical evidence for the G1 Simpson rework.

This is an evidence script, not production code and not a test.  It prints a
single JSON object containing:

* hand-parameter checks of the ``2*d`` complete-panel width for several even
  subinterval counts (including ``n=2``);
* the analytic double-lane-change path budget reconstructed from raw panel
  arithmetic, its refinement record and representative partial queries;
* a legal C2 profile with non-zero fourth-derivative term, its corrected
  budget, and position errors against independent adaptive quadrature.

Run from the worktree root with bytecode disabled.  The companion log is
``logs/budget_evidence.log``.
"""
import datetime
import importlib.util
import json
import math
from pathlib import Path
import sys

RUN_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.continuous_geometry import ControlKnot  # noqa: E402
from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    C2CurvatureProfile, VirtualAPath)
from fwis_v3_optimizer.formation_continuous_a import bounds  # noqa: E402


def _load_benchmark():
    path = REPO_ROOT / "tools" / "virtual_a_benchmark.py"
    spec = importlib.util.spec_from_file_location("virtual_a_benchmark", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BENCHMARK = _load_benchmark()


def _research_model():
    return FormationModel((
        VehicleOffset(1, 1.0, 0.0),
        VehicleOffset(2, 0.0, 1.0),
        VehicleOffset(3, 0.0, -1.0)))


def _panel_remainder(width, m4):
    return width ** 5 * m4 / 2880.0


def _hand_parameter_checks():
    m4 = 5.5
    rows = []
    for h, n in ((2.0, 2), (1.7, 4), (3.0, 10), (0.8, 200), (6.0, 600)):
        record = bounds.simpson_segment_budget(h, n, m4)
        d = h / n
        width = 2.0 * d
        expected_full = (n // 2) * _panel_remainder(width, m4)
        expected_local = _panel_remainder(width, m4)
        old = (n + 1) * _panel_remainder(d, m4)
        rows.append({
            "h_m": h,
            "subintervals_n": n,
            "subinterval_width_d_m": d,
            "required_full_panel_width_2d_m": width,
            "reported_full_panel_width_m": record["full_panel_width_m"],
            "full_panel_count": record["full_panel_count"],
            "full_remainder_m": record["full_remainder_m"],
            "local_panel_count": record["local_panel_count"],
            "local_panel_width_m": record["local_panel_width_m"],
            "local_remainder_m": record["local_remainder_m"],
            "segment_remainder_m": record["segment_remainder_m"],
            "expected_full_remainder_m": expected_full,
            "expected_local_remainder_m": expected_local,
            "expected_segment_remainder_m": expected_full + expected_local,
            "old_n_plus_one_d_budget_m": old,
            "corrected_over_old_ratio": (
                record["segment_remainder_m"] / old),
            "old_formula_is_underestimate": record["segment_remainder_m"] > old,
            "full_width_is_2d": (
                record["full_panel_width_m"] >= width
                and record["full_panel_width_m"] <= width * (1.0 + 1e-12)),
        })
    return {"m4": m4, "cases": rows}


def _raw_recompute(profile, path):
    """Reproduce the reported budget from raw panels, not from the helper."""
    budget = path.integration_budget
    per_coordinate = 0.0
    full_total = 0
    local_total = 0
    rows = []
    for index, (s0, s1) in enumerate(profile.domain_segments):
        h = s1 - s0
        n = budget["panels_per_segment"][index]
        m4 = budget["m4_per_segment"][index]
        d = h / n
        width = 2.0 * d
        full_count = n // 2
        full = full_count * _panel_remainder(width, m4)
        local = _panel_remainder(width, m4)
        per_coordinate += full + local
        full_total += full_count
        local_total += 1
        rows.append({
            "segment": index,
            "h_m": h,
            "subintervals_n": n,
            "subinterval_width_d_m": d,
            "full_panel_width_2d_m": width,
            "full_panel_count": full_count,
            "reported_full_panel_width_m":
                budget["simpson_panel_widths_m"][index],
            "full_remainder_m": full,
            "reported_full_remainder_m":
                budget["simpson_full_remainders_m"][index],
            "local_remainder_m": local,
            "reported_local_remainder_m":
                budget["simpson_local_remainders_m"][index],
        })
    euclidean = math.sqrt(2.0) * per_coordinate
    guard = bounds.POSITION_ROUNDOFF_GUARD_M * (
        full_total + local_total + 8)
    guard += path.length_m * budget["angle_error_bound_rad"]
    total = euclidean + guard
    return {
        "rows": rows,
        "recomputed_per_coordinate_m": per_coordinate,
        "reported_per_coordinate_m": budget["simpson_per_coordinate_m"],
        "recomputed_euclidean_m": euclidean,
        "reported_euclidean_m": budget["simpson_euclidean_m"],
        "recomputed_guard_m": guard,
        "reported_guard_m": budget["floating_point_guard_m"],
        "recomputed_total_m": total,
        "reported_total_m": budget["position_error_bound_m"],
        "reported_total_leq_1e_7": budget["position_error_bound_m"] <= 1e-7,
        "reported_total_equals_recompute": (
            abs(budget["position_error_bound_m"] - total)
            <= 1e-12 * max(total, 1e-300)),
    }


def _budget_copy(path):
    budget = path.integration_budget
    keys = (
        "kind", "position_error_bound_m", "angle_error_bound_rad",
        "total_panels", "total_full_simpson_panels",
        "local_simpson_panels_per_segment", "total_local_simpson_panels",
        "simpson_euclidean_m", "simpson_per_coordinate_m",
        "simpson_panel_widths_m", "simpson_local_panel_widths_m",
        "simpson_full_remainders_m", "simpson_local_remainders_m",
        "simpson_segment_remainders_m", "floating_point_guard_m",
        "panels_per_segment", "initial_panels_per_segment",
        "refinement_steps", "m4_per_segment", "panel_widths_m",
        "subinterval_widths_m", "numerics_version")
    return {key: budget[key] for key in keys}


def _analytic_evidence(model):
    profile = BENCHMARK.DoubleLaneChangeProfile()
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile, model)
    audit = BENCHMARK._integration_budget_audit(profile, path)
    partial = BENCHMARK._partial_query_evidence(profile, path)
    worst = max(row["quadrature_error_m"]
                for row in partial["stations"])
    return {
        "profile": "analytic_sin3_double_lane_change",
        "budget": _budget_copy(path),
        "raw_recompute": _raw_recompute(profile, path),
        "benchmark_audit": audit,
        "representative_partial_queries": partial,
        "representative_worst_quadrature_error_m": worst,
        "representative_all_within_bound": (
            partial["all_within_bound"]
            and worst <= path.position_error_bound_m),
    }


def _c2_evidence(model):
    profile = C2CurvatureProfile.from_knots((
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(1.0, 0.0, 0.1),
        ControlKnot(2.0, 0.0, -0.4)))
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile, model)
    audit = BENCHMARK._integration_budget_audit(profile, path)
    stations = []
    for s in (0.13, 0.47, 0.999999, 1.0, 1.000001, 1.5, 1.999999):
        x, y = BENCHMARK.quadrature_position(profile, s)
        state = path.state_at_s(s)
        error = math.hypot(state.qf.x - x, state.qf.y - y)
        stations.append({
            "s": s,
            "quadrature_x_m": x,
            "quadrature_y_m": y,
            "production_x_m": state.qf.x,
            "production_y_m": state.qf.y,
            "position_error_m": error,
            "reported_total_budget_m": path.position_error_bound_m,
            "within_reported_budget": (
                error <= path.position_error_bound_m),
            "margin_m": path.position_error_bound_m - error,
        })
    return {
        "profile": "c2_cubic_hermite_nonzero_g",
        "knots": [(knot.s, knot.value, knot.derivative)
                  for knot in profile.knots],
        "g_at_0_25": profile.jet_at(0.25).g,
        "g_at_1_75": profile.jet_at(1.75).g,
        "budget": _budget_copy(path),
        "raw_recompute": _raw_recompute(profile, path),
        "benchmark_audit": audit,
        "quadrature_method": {
            "routine": "scipy.integrate.quad",
            "integrand": "cos(psi(s)), sin(psi(s))",
            "epsabs": 1e-13,
            "epsrel": 1e-13,
            "limit": 400,
            "break_points": "profile knots strictly inside [0, s]",
        },
        "stations": stations,
        "worst_position_error_m": max(
            row["position_error_m"] for row in stations),
        "all_within_reported_budget": all(
            row["within_reported_budget"] for row in stations),
    }


def main():
    model = _research_model()
    evidence = {
        "schema": "virtual_rigid_a.g1.budget_evidence.v2",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN_ROOT.name,
        "numerics_version": bounds.NUMERICS_VERSION,
        "formulas": {
            "per_panel_remainder": "H^5 * M4 / 2880",
            "complete_segment": "(n/2) * R(2*d)",
            "local_query": "one panel with width < 2*d, conservatively R(2*d)",
            "per_coordinate": "sum over segments of full + local",
            "position_combination": "sqrt(2) * per_coordinate",
            "floating_guard": (
                "1e-10 * (total_full_panels + total_local_panels + 8) "
                "+ path_length * angle_bound"),
            "total": "simpson_euclidean + floating_guard",
            "d": "h/n for even n",
        },
        "hand_parameter_checks": _hand_parameter_checks(),
        "analytic_double_lane_change": _analytic_evidence(model),
        "c2_nonzero_g": _c2_evidence(model),
        "nonclaims": [
            "sampled quadrature differences are numerical evidence, not a continuous-domain proof",
            "no expensive refinement or NLP solve was performed",
            "Codex acceptance is NOT_RUN_NOT_CLAIMED",
        ],
    }
    print(json.dumps(evidence, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()