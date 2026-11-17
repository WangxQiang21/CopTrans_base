#!/usr/bin/env python3
"""C0-B same-candidate budget analysis and float-error check.

Reads the frozen candidates written by ``capture_c_candidates.py`` and:

1. recomputes the pre-fix G1 budget decomposition the audit rejected at
   ``9.95e-8..1.02e-7`` (Simpson remainder, ``bounds.position_guard``, angle
   term, panel counts, total);
2. recomputes the same-candidate budget with the implemented
   ``integration_bounds.evaluation_guard`` and the ``sqrt(2)`` Euclidean
   combination, at the frozen initial panels and at the 64-step refined
   panels;
3. compares the audit double-precision Simpson positions with an independent
   extended-precision (numpy longdouble) reference at the same panel counts
   (float64 evaluation error only) and at 1024 subintervals per piece
   (evaluation plus truncation);
4. compares the production ``VirtualAPath.state_at_s`` positions with the
   same 1024-subinterval reference.

Writes only ``budget_analysis.json`` and stdout next to this script.
"""
import json
import math
from pathlib import Path

import numpy as np

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation_continuous_a import audit as audit_module
from fwis_v3_optimizer.formation_continuous_a import bounds
from fwis_v3_optimizer.formation_continuous_a import integration_bounds
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GeometryAuditConfig,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)

import capture_c_candidates as capture

HERE = Path(__file__).resolve().parent
CASES = capture.CASES
REFERENCE_SUBINTERVALS = 1024
LD = np.longdouble


def _piece_psi(piece, psi0, q):
    """psi(q) evaluated in extended precision from the float coefficients."""
    h = LD(piece.h)
    c0 = LD(piece.c0)
    c1 = LD(piece.c1)
    c2 = LD(piece.c2)
    c3 = LD(piece.c3)
    return LD(psi0) + h * q * (
        c0 + q * (c1 / LD(2) + q * (c2 / LD(3) + q * c3 / LD(4))))


def _reference_piece_displacement(piece, psi0, qa, qb,
                                  subintervals=REFERENCE_SUBINTERVALS):
    """Extended-precision composite Simpson with pairwise summation."""
    count = int(subintervals)
    q = np.linspace(LD(qa), LD(qb), count + 1)
    psi = _piece_psi(piece, psi0, q)
    weights = np.full(count + 1, LD(2), dtype=LD)
    weights[0] = LD(1)
    weights[-1] = LD(1)
    weights[1:-1:2] = LD(4)
    cosine = np.sum(weights * np.cos(psi))
    sine = np.sum(weights * np.sin(psi))
    factor = LD(piece.h) * (LD(qb) - LD(qa)) / (LD(3) * LD(count))
    return float(factor * cosine), float(factor * sine)


def _reference_position(pieces, start, station):
    """Extended-precision position at ``station``, piece-wise."""
    x = LD(start.x)
    y = LD(start.y)
    psi = LD(start.psi)
    remaining = LD(station)
    for piece in pieces:
        if remaining <= 0.0:
            break
        length = LD(piece.h)
        fraction = min(LD(1), remaining / length)
        dx, dy = _reference_piece_displacement(
            piece, psi, LD(0), fraction)
        x += LD(dx)
        y += LD(dy)
        psi += _piece_psi(piece, LD(0), fraction)
        remaining -= length * fraction
        if fraction < 1.0:
            break
    return float(x), float(y)


def _same_n_piece_error(pieces, subintervals_by_piece):
    """Isolate double float error: audit vs longdouble at the same n."""
    worst = 0.0
    rows = []
    for index, piece in enumerate(pieces):
        count = subintervals_by_piece[index]
        psi0 = 0.0
        dx, dy = audit_module._simpson_piece_integral(
            piece, psi0, 0.0, 1.0, count)
        ref_dx, ref_dy = _reference_piece_displacement(
            piece, LD(0), LD(0), LD(1), count)
        error = math.hypot(dx - ref_dx, dy - ref_dy)
        worst = max(worst, error)
        rows.append({
            "segment_index": index,
            "subintervals": count,
            "audit_x": dx, "audit_y": dy,
            "same_n_reference_x": ref_dx, "same_n_reference_y": ref_dy,
            "error_m": error,
        })
    return worst, rows


def _audit_error(pieces, start, stations, subintervals_by_piece):
    """Max |audit Simpson - extended reference| for the given panel plan."""
    prefixes = audit_module._psi_prefixes(pieces)
    worst = 0.0
    per_station = []
    for station in stations:
        x, y = audit_module._integrate_xy(
            pieces, prefixes, start, station, subintervals_by_piece)
        ref_x, ref_y = _reference_position(pieces, start, station)
        error = math.hypot(x - ref_x, y - ref_y)
        worst = max(worst, error)
        per_station.append({
            "s": float(station),
            "audit_x": x, "audit_y": y,
            "reference_x": ref_x, "reference_y": ref_y,
            "error_m": error,
        })
    return worst, per_station


def _production_error(path, pieces, stations):
    worst = 0.0
    rows = []
    for station in stations:
        state = path.state_at_s(station)
        ref_x, ref_y = _reference_position(pieces, path.start_qf, station)
        error = math.hypot(state.qf.x - ref_x, state.qf.y - ref_y)
        worst = max(worst, error)
        rows.append({"s": float(station), "error_m": error})
    return worst, rows


def _stations(pieces):
    stations = []
    for piece in pieces:
        for fraction in (0.0, 0.13, 0.37, 0.5, 0.71, 0.89, 1.0):
            station = piece.s0 + piece.h * fraction
            if station > 0.0 and (
                    not stations or station != stations[-1]):
                stations.append(station)
    return stations


def _guard_records(segments, pieces):
    """``(h, subintervals, sum|c|)`` records for the implemented guard."""
    return [
        (float(segment["h"]), int(segment["subintervals"]),
         abs(piece.c0) + abs(piece.c1) + abs(piece.c2) + abs(piece.c3))
        for segment, piece in zip(segments, pieces)]


def _audit_guard(segments, pieces, plan, start_psi, start_magnitude):
    """The implemented per-coordinate and Euclidean audit guard."""
    evaluation = integration_bounds.evaluation_guard(
        _guard_records(segments, pieces), start_psi, start_magnitude)
    angle_guard = bounds._next_up(
        pieces[-1].s1 * plan["angle_bound_rad"])
    per_coordinate = bounds._next_up(
        evaluation["total_evaluation_m"] + angle_guard)
    euclidean = bounds._next_up(
        bounds._next_up(math.sqrt(2.0)) * per_coordinate)
    return {
        "evaluation_guard_m": evaluation["total_evaluation_m"],
        "evaluation_components": evaluation,
        "angle_guard_m": angle_guard,
        "per_coordinate_guard_m": per_coordinate,
        "euclidean_guard_m": euclidean,
    }


def _new_budget(segments, pieces, plan, start_psi, start_magnitude):
    guard = _audit_guard(
        segments, pieces, plan, start_psi, start_magnitude)
    total = bounds._next_up(
        plan["euclidean_remainder_m"] + guard["euclidean_guard_m"])
    return {
        "simpson_euclidean_remainder_m": plan["euclidean_remainder_m"],
        "evaluation_guard_m": guard["evaluation_guard_m"],
        "psi_accumulation_m":
            guard["evaluation_components"]["psi_accumulation_m"],
        "coordinate_accumulation_m":
            guard["evaluation_components"]["coordinate_accumulation_m"],
        "psi_bound_rad": guard["evaluation_components"]["psi_bound_rad"],
        "angle_guard_m": guard["angle_guard_m"],
        "per_coordinate_guard_m": guard["per_coordinate_guard_m"],
        "euclidean_guard_m": guard["euclidean_guard_m"],
        "total_budget_m": total,
        "margin_to_5e8_m": 5e-8 - total,
        "confirms_5e8": bool(total <= 5e-8),
    }


def analyse(candidate, start, model, config):
    knot_objects = [
        ControlKnot(row["s"], row["kappa"], row["eta"])
        for row in candidate["profile"]["knots"]]
    pieces, _stations_values, _values, _derivatives = \
        audit_module._independent_pieces(knot_objects)
    path = VirtualAPath(
        start, C2CurvatureProfile.from_knots(knot_objects), model)
    decomposition = candidate["decomposition"]
    initial = decomposition["initial"]
    final = decomposition["final"]
    initial_subintervals = [seg["subintervals"] for seg in initial["segments"]]
    final_subintervals = [seg["subintervals"] for seg in final["segments"]]
    start_magnitude = max(abs(start.x), abs(start.y))
    stations = _stations(pieces)
    initial_worst, initial_rows = _audit_error(
        pieces, start, stations, initial_subintervals)
    final_worst, final_rows = _audit_error(
        pieces, start, stations, final_subintervals)
    production_worst, production_rows = _production_error(
        path, pieces, stations)
    same_n_worst, same_n_rows = _same_n_piece_error(
        pieces, initial_subintervals)
    same_n_final_worst, same_n_final_rows = _same_n_piece_error(
        pieces, final_subintervals)
    initial_new = _new_budget(
        initial["segments"], pieces, initial, start.psi, start_magnitude)
    final_new = _new_budget(
        final["segments"], pieces, final, start.psi, start_magnitude)
    return {
        "capture_index": candidate["capture_index"],
        "length_m": float(path.length_m),
        "segment_count": len(pieces),
        "initial_subintervals": initial_subintervals,
        "final_subintervals": final_subintervals,
        "initial_budget": {
            "simpson_euclidean_remainder_m":
                initial["euclidean_remainder_m"],
            "old_roundoff_guard_m": initial["roundoff_term_m"],
            "old_angle_guard_m": initial["angle_guard_term_m"],
            "old_total_budget_m": initial["total_budget_m"],
            "old_confirms_5e8": bool(initial["total_budget_m"] <= 5e-8),
            "new_": initial_new,
        },
        "final_budget": {
            "simpson_euclidean_remainder_m":
                final["euclidean_remainder_m"],
            "old_roundoff_guard_m": final["roundoff_term_m"],
            "old_angle_guard_m": final["angle_guard_term_m"],
            "old_total_budget_m": final["total_budget_m"],
            "new_": final_new,
        },
        "refinement": {
            "minimum_old_total_budget_m":
                decomposition["minimum"]["total_budget_m"],
            "minimum_old_refinement_steps":
                decomposition["minimum"]["refinement_steps"],
            "final_old_total_budget_m": final["total_budget_m"],
            "final_old_refinement_steps": final["refinement_steps"],
            "final_termination": final.get("termination", "CONFIRMED"),
            "old_guard_grew_over_refinement": bool(
                final["roundoff_term_m"] > initial["roundoff_term_m"]),
        },
        "float_error": {
            "reference": "numpy longdouble composite Simpson, 1024 "
                         "subintervals per piece, pairwise summation",
            "audit_initial_worst_m": initial_worst,
            "audit_final_worst_m": final_worst,
            "production_state_at_s_worst_m": production_worst,
            "same_n_audit_initial_worst_m": same_n_worst,
            "same_n_audit_final_worst_m": same_n_final_worst,
            "new_guard_over_same_n_float_error":
                (initial_new["evaluation_guard_m"]
                 / max(same_n_worst, 1e-300)),
            "new_total_over_measured_full_error":
                initial_new["total_budget_m"] / max(initial_worst, 1e-300),
            "old_guard_over_same_n_float_error":
                (initial["roundoff_term_m"] / max(same_n_worst, 1e-300)),
        },
        "initial_rows": initial_rows,
        "final_rows": final_rows,
        "production_rows": production_rows,
        "same_n_rows": same_n_rows,
        "same_n_final_rows": same_n_final_rows,
    }


def main():
    source = json.loads((HERE / "candidates.json").read_text(encoding="utf-8"))
    spec = json.loads(CASES.read_text(encoding="utf-8"))
    map_request, _identity = capture.build_case(spec, "C")
    start = map_request.request.start_qf
    model = map_request.request.model
    results = []
    for candidate in source["candidates"]:
        result = analyse(candidate, start, model, GeometryAuditConfig())
        results.append(result)
        print(json.dumps({
            "capture_index": result["capture_index"],
            "old_initial_total": result["initial_budget"][
                "old_total_budget_m"],
            "old_minimum_total": result["refinement"][
                "minimum_old_total_budget_m"],
            "old_final_total": result["refinement"][
                "final_old_total_budget_m"],
            "new_initial_total": result["initial_budget"]["new_"][
                "total_budget_m"],
            "new_final_total": result["final_budget"]["new_"][
                "total_budget_m"],
            "new_confirms": result["initial_budget"]["new_"][
                "confirms_5e8"],
            "new_evaluation_guard": result["initial_budget"]["new_"][
                "evaluation_guard_m"],
            "new_euclidean_guard": result["initial_budget"]["new_"][
                "euclidean_guard_m"],
            "audit_initial_worst": result["float_error"][
                "audit_initial_worst_m"],
            "audit_final_worst": result["float_error"][
                "audit_final_worst_m"],
            "production_worst": result["float_error"][
                "production_state_at_s_worst_m"],
            "same_n_float_worst": result["float_error"][
                "same_n_audit_initial_worst_m"],
            "same_n_float_worst_final": result["float_error"][
                "same_n_audit_final_worst_m"],
            "new_guard_over_same_n_float": result["float_error"][
                "new_guard_over_same_n_float_error"],
            "new_total_over_measured_error": result["float_error"][
                "new_total_over_measured_full_error"],
        }, sort_keys=True), flush=True)
    output = {
        "schema": "virtual_a_corridor.c0b.budget_analysis.v2",
        "reference_subintervals": REFERENCE_SUBINTERVALS,
        "float64_epsilon": bounds.EPSILON,
        "fixed_threshold_m": 5e-8,
        "production_guard_unchanged_m_per_panel":
            bounds.POSITION_ROUNDOFF_GUARD_M,
        "candidates": results,
    }
    destination = HERE / "budget_analysis.json"
    destination.write_text(
        json.dumps(output, indent=1, sort_keys=True) + "\n",
        encoding="utf-8")
    print("WROTE", destination, flush=True)


if __name__ == "__main__":
    main()