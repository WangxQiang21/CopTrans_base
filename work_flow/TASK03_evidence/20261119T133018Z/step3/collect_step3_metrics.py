#!/usr/bin/env python3
"""TASK03 Step 3 evidence: measured audit and one-cut refinement behaviour.

The assertions live in ``FormationSampleAuditStep3Tests`` in
``tests/test_virtual_a_formation_corridor.py``.  This script records the same
measured numbers: a real accepted union candidate and its independent sample
audit, a direct definite corner violation, an unconfirmed near-boundary case,
the F-feasible/complete-OBB-out numeric rows, and the spy-proven second-round
cut rows inside numeric and CasADi ``g``.

Run from the worktree root with the TASK03 plan's PYTHONPATH.
"""
import json
import os
import sys
import traceback
from unittest import mock

import numpy as np

ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from fwis_v3_optimizer.formation_continuous_a import optimizer as O  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (  # noqa: E402
    audit_formation_template_samples,
)
from tests.test_virtual_a_formation_corridor import (  # noqa: E402
    CONTEXT_SHA,
    INTERVALS,
    MARGIN_M,
    S_LENGTH,
    _audit_payload,
    _build_casadi_nlp,
    _build_corridor_constraint,
    _constraint_value_offsets,
    _fake_ipopt_result,
    _numeric_constraint_vector,
    _scene,
    _search_packet,
    _step3_scene,
    _targeted_samples,
    _z,
    solve_open_path_guided,
)

try:
    import casadi
except ImportError:  # recorded as NOT_RUN in the evidence, never faked
    casadi = None


def _audit_summary(audit):
    witness = audit.get("witness")
    return {
        "status": audit.get("status"),
        "reason": audit.get("reason"),
        "sample_count": int(audit.get("sample_count", 0)),
        "planned_sample_count": int(
            audit.get("planned_sample_count", 0)),
        "spacing_sigma": audit.get("spacing_sigma"),
        "max_residual_m": audit.get("max_residual_m"),
        "max_evidence_m": audit.get("max_evidence_m"),
        "cut_sigma": audit.get("cut_sigma"),
        "is_continuous_certificate": bool(
            audit.get("is_continuous_certificate", False)),
        "witness": None if witness is None else {
            "sigma": witness.get("sigma"),
            "vehicle_id": witness.get("vehicle_id"),
            "region_index": witness.get("region_index"),
            "halfspace_id": witness.get("halfspace_id"),
            "vertex_id": witness.get("vertex_id"),
            "residual_m": witness.get("residual_m"),
            "error_budget_m": witness.get("error_budget_m"),
            "evidence_m": witness.get("evidence_m"),
        },
    }


def _constraint_summary(constraint):
    formation = constraint.formation
    return {
        "formation_sample_count": int(formation.sample_count),
        "formation_scalar_row_count": int(formation.scalar_row_count),
        "formation_rows_by_vehicle": [
            [int(vehicle_id), int(count)]
            for vehicle_id, count in formation.rows_by_vehicle],
        "formation_segment_row_counts": [
            int(count) for count in formation.segment_row_counts],
        "f_halfspace_sample_count": int(constraint.sample_count),
        "targeted_sigmas": sorted({
            round(float(sample.sigma), 12)
            for sample in _targeted_samples(constraint)}),
        "targeted_sample_count": len(_targeted_samples(constraint)),
    }


def _real_clear():
    scene = _scene()
    result = solve_open_path_guided(
        scene["request"], scene["config"],
        route_guides=(scene["guide"],),
        corridor_chain=scene["chain"],
        corridor_allocation=scene["allocation"],
        formation_template=scene["template"],
        formation_planning_context_sha256=CONTEXT_SHA)
    accepted = result.attempts[-1]
    return {
        "status": result.status,
        "audit_passed": bool(result.audit.passed if result.audit else False),
        "sample_audit": _audit_summary(result.formation_sample_audit),
        "refinement": result.formation_refinement,
        "formation_obb_constraint_count": (
            result.formation_obb_constraint_count),
        "formation_obb_max_residual_m": (
            result.formation_obb_max_residual_m),
        "corridor_halfspace_constraint_count": (
            result.corridor_halfspace_constraint_count),
        "corridor_max_residual_m": result.corridor_max_residual_m,
        "variables": {
            "vector_length": int(
                1 + len(accepted.initial.kappas)
                + len(accepted.initial.etas)),
            "hermite_segments": int(len(accepted.initial.kappas) - 1),
            "S_m": float(accepted.initial.S),
        },
        "attempt_statuses": [
            attempt.status for attempt in result.attempts],
        "note": (
            "real IPOPT candidate; the audit is finite samples only and is "
            "not a continuous certificate"),
    }


def _direct_definite():
    scene = _step3_scene(y_half=1.1)
    audit = audit_formation_template_samples(
        scene["path"], scene["template"], scene["chain"])

    request = scene["request"]
    config = scene["config"]
    packet = _search_packet(request, config)
    constraint = _build_corridor_constraint(
        scene["chain"], scene["allocation"], request, config,
        formation_template=scene["template"],
        planning_context_sha256=CONTEXT_SHA,
        formation_guide=scene["guide"])
    _eq_end, signed_end, corridor_end = _constraint_value_offsets(
        request, config, packet, constraint)
    values = _numeric_constraint_vector(
        _z(0.0), request, config, packet,
        int(config.collocation_points),
        corridor_constraint=constraint)
    formation_count = constraint.formation.scalar_row_count
    f_rows = values[signed_end:corridor_end - formation_count]
    obb_rows = values[corridor_end - formation_count:corridor_end]
    return {
        "audit": _audit_summary(audit),
        "f_feasible_obb_out_numeric_rows": {
            "f_halfspace_max_residual_m": float(np.max(f_rows)),
            "complete_obb_max_residual_m": float(np.max(obb_rows)),
            "formation_row_count": int(formation_count),
            "numeric_vector_size": int(values.size),
            "corridor_end": int(corridor_end),
        },
        "note": (
            "the straight F reference path satisfies every old signed/F "
            "halfspace while complete vehicle corners exceed the wall"),
    }


def _direct_unconfirmed():
    y_half = 1.25 + MARGIN_M - 5e-10
    scene = _step3_scene(y_half=y_half)
    audit = audit_formation_template_samples(
        scene["path"], scene["template"], scene["chain"])
    return {
        "audit": _audit_summary(audit),
        "y_half_m": float(y_half),
        "note": (
            "positive residual below the 1e-9 m definite threshold yields "
            "UNCONFIRMED and no cut"),
    }


def _mocked_one_cut():
    scene = _scene()
    request = scene["request"]
    config = scene["config"]
    calls = []
    witness = {
        "sigma": 0.3, "s_m": 0.3 * S_LENGTH,
        "segment_index": 0, "vehicle_id": 2, "decomposition": "whole",
        "body_piece_index": 0, "region_index": 0, "halfspace_id": 3,
        "vertex_id": 1, "point_x": 1.0, "point_y": 2.0,
        "residual_m": 1.0, "error_budget_m": 0.0, "evidence_m": 1.0,
    }
    audits = iter([
        _audit_payload(
            scene["template"], "DEFINITE_VIOLATION",
            reason="definite OBB violation", cut_sigma=0.3,
            witness=witness),
        _audit_payload(
            scene["template"], "SAMPLED_CLEAR", reason=""),
    ])

    def fake_audit(path, template, chain):
        return next(audits)

    def fake_solve(request, config, packet, guess,
                   wall_limit_s=None, corridor_constraint=None, **kwargs):
        calls.append((guess, corridor_constraint))
        return _fake_ipopt_result(_z(0.0))

    with mock.patch.object(
            O, "_solve_with_ipopt", side_effect=fake_solve), \
            mock.patch.object(
                O, "_formation_sample_audit", side_effect=fake_audit):
        result = solve_open_path_guided(
            request, config,
            route_guides=(scene["guide"],),
            corridor_chain=scene["chain"],
            corridor_allocation=scene["allocation"],
            formation_template=scene["template"],
            formation_planning_context_sha256=CONTEXT_SHA)
    initial_constraint = calls[0][1]
    refined_constraint = calls[1][1]
    initial_summary = _constraint_summary(initial_constraint)
    refined_summary = _constraint_summary(refined_constraint)

    packet = _search_packet(request, config)
    _eq, signed_end, corridor_end = _constraint_value_offsets(
        request, config, packet, refined_constraint)
    f_rows = sum(
        len(limits) for group in refined_constraint.groups
        for _q, _rows, limits in group)
    numeric = _numeric_constraint_vector(
        _z(0.0), request, config, packet,
        int(config.collocation_points),
        corridor_constraint=refined_constraint)
    symbolic = None
    if casadi is not None:
        captured = {}

        def fake_nlpsol(name, plugin, nlp, options=None):
            captured["nlp"] = nlp
            return object()

        with mock.patch.object(
                casadi, "nlpsol", side_effect=fake_nlpsol):
            _build_casadi_nlp(
                request, config, packet,
                corridor_constraint=refined_constraint)
        g_function = casadi.Function(
            "g", [captured["nlp"]["x"]], [captured["nlp"]["g"]])
        symbolic_values = np.asarray(
            g_function(_z(0.0)), dtype=float).ravel()
        symbolic = {
            "size": int(symbolic_values.size),
            "matches_numeric": bool(np.allclose(
                symbolic_values, numeric, atol=1e-8, rtol=0.0)),
        }
    return {
        "status": result.status,
        "solver_call_count": len(calls),
        "initial_guess_variables": {
            "vector_length": int(len(calls[0][0].vector)),
            "hermite_segments": int(len(calls[0][0].kappas) - 1),
        },
        "refinement_guess_variables": {
            "vector_length": int(len(calls[1][0].vector)),
            "source": calls[1][0].source,
            "label": calls[1][0].label,
        },
        "initial_constraint": initial_summary,
        "refined_constraint": refined_summary,
        "added": {
            "samples": (
                refined_summary["formation_sample_count"]
                - initial_summary["formation_sample_count"]),
            "rows": (
                refined_summary["formation_scalar_row_count"]
                - initial_summary["formation_scalar_row_count"]),
        },
        "g_rows": {
            "signed_end": int(signed_end),
            "corridor_end": int(corridor_end),
            "f_halfspace_rows": int(f_rows),
            "numeric_size": int(numeric.size),
            "numeric_size_equals_signed_plus_f_plus_obb": bool(
                numeric.size == signed_end + f_rows
                + refined_summary["formation_scalar_row_count"]),
            "casadi": symbolic,
        },
        "refinement": result.formation_refinement,
        "attempt_statuses": [
            attempt.status for attempt in result.attempts],
        "note": (
            "the solver is a deterministic double; the constraint rebuild, "
            "numeric vector and CasADi g are the production code"),
    }


def _real_obb_infeasible():
    scene = _step3_scene(y_half=1.1, max_seconds=3.0)
    result = solve_open_path_guided(
        scene["request"], scene["config"],
        route_guides=(scene["guide"],),
        corridor_chain=scene["chain"],
        corridor_allocation=scene["allocation"],
        formation_template=scene["template"],
        formation_planning_context_sha256=CONTEXT_SHA)
    return {
        "status": result.status,
        "reason": result.reason,
        "path_is_none": result.path is None,
        "accepted_attempts": [
            attempt.status for attempt in result.attempts
            if attempt.status == "SOLVER_SUCCESS_AUDITED"],
        "attempt_statuses": [
            attempt.status for attempt in result.attempts],
        "formation_obb_max_residual_m": (
            result.formation_obb_max_residual_m),
        "deadline_summary": result.deadline_summary,
        "note": (
            "two walls 2.2 m apart cannot contain the 2.5 m complete "
            "three-vehicle OBB span; no candidate is accepted"),
    }


def main():
    payload = {
        "role": "task03_step3_measured_sample_audit_and_one_cut",
        "worktree": ROOT,
        "intervals": int(INTERVALS),
        "real_clear_scene": _real_clear(),
        "direct_definite_violation": _direct_definite(),
        "direct_unconfirmed_near_boundary": _direct_unconfirmed(),
        "mocked_one_audit_directed_cut": _mocked_one_cut(),
        "real_f_feasible_obb_infeasible": _real_obb_infeasible(),
        "casadi_available": casadi is not None,
        "dialect": (
            "row counts are actual appended scalar inequalities; statuses "
            "are measured, not inferred"),
    }
    json.dump(payload, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        traceback.print_exc()
        raise