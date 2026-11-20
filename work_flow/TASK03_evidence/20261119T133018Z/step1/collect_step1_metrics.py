#!/usr/bin/env python3
"""TASK03 Step 1 evidence: measured OBB/g metrics on the frozen fixture.

The assertions live in ``tests/test_virtual_a_formation_corridor.py``; this
script records the actual maxima and counts so the acceptance evidence does
not rely on assertions alone.  It imports the test fixture helpers on purpose:
the values must come from the same scene the tests exercise, not a second
hand-copied scene.

Run from the repository root with the PYTHONPATH from TASK03 plan section 7.
"""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import casadi  # noqa: E402  (evidence requires the real solver stack)

from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a.corridor import (  # noqa: E402
    obb_polygon,
    obb_support_halfwidth,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (  # noqa: E402
    _build_casadi_nlp,
    _constraint_value_offsets,
    _numeric_constraint_vector,
    _search_packet,
    solve_open_path_guided,
)
from tests.test_virtual_a_formation_corridor import (  # noqa: E402
    CONTEXT_SHA,
    INTERVALS,
    S_LENGTH,
    _CorridorWrapper,
    _casadi_nlp,
    _independent_rows,
    _scene,
    _z,
    replace_config_scope,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
)


def geometry_metrics():
    combinations = ((0.0, 0.1), (0.7, -0.1), (-1.1, 0.1), (2.6, -0.1))
    rows = []
    for psi0, kappa in combinations:
        scene = _scene(psi0=psi0, kappa=kappa)
        request = scene["request"]
        constraint = scene["constraint"]
        path = scene["path"]
        cosine, sine = math.cos(psi0), math.sin(psi0)
        center_error = heading_error = support_error = polygon_error = 0.0
        for group in constraint.groups:
            for sample in group:
                state = path.state_at_s(sample.sigma * S_LENGTH)
                vehicle = [
                    item for item in state.vehicles
                    if item.vehicle_id == sample.vehicle_id][0]
                qf = state.qf
                kappa_at = state.curvature.kappa
                forward = 1.0 - sample.offset_y * kappa_at
                lateral = sample.offset_x * kappa_at
                theta = qf.psi + math.atan2(lateral, forward)
                center_x = (qf.x + math.cos(qf.psi) * sample.offset_x
                            - math.sin(qf.psi) * sample.offset_y)
                center_y = (qf.y + math.sin(qf.psi) * sample.offset_x
                            + math.cos(qf.psi) * sample.offset_y)
                local_x = cosine * center_x + sine * center_y
                local_y = -sine * center_x + cosine * center_y
                local_theta = theta - psi0
                center_error = max(center_error, math.hypot(
                    local_x - (cosine * vehicle.x + sine * vehicle.y),
                    local_y - (-sine * vehicle.x + cosine * vehicle.y)))
                heading_error = max(
                    heading_error, abs(local_theta - (vehicle.theta - psi0)))
                shape = request.vehicle_shapes[sample.vehicle_id]
                corners = [vertex.local_vertex for vertex in sample.vertices]
                actual_polygon = tuple(
                    (center_x + math.cos(theta) * px - math.sin(theta) * py,
                     center_y + math.sin(theta) * px + math.cos(theta) * py)
                    for px, py in corners)
                reference_polygon = obb_polygon(
                    (center_x, center_y), theta, shape)
                for actual, reference in zip(actual_polygon,
                                              reference_polygon):
                    polygon_error = max(
                        polygon_error,
                        math.hypot(actual[0] - reference[0],
                                   actual[1] - reference[1]))
                for halfspace_id in range(len(sample.vertices[0].halfspaces)):
                    halfspace = sample.vertices[0].halfspaces[halfspace_id]
                    projections = []
                    for vertex in sample.vertices:
                        px, py = vertex.local_vertex
                        corner_x = (local_x + math.cos(local_theta) * px
                                    - math.sin(local_theta) * py)
                        corner_y = (local_y + math.sin(local_theta) * px
                                    + math.cos(local_theta) * py)
                        projections.append(halfspace.row[0] * corner_x
                                           + halfspace.row[1] * corner_y)
                    expected = (halfspace.row[0] * local_x
                                + halfspace.row[1] * local_y
                                + obb_support_halfwidth(
                                    halfspace.row, local_theta, shape))
                    support_error = max(support_error,
                                        abs(max(projections) - expected))
        rows.append({
            "psi0_rad": psi0,
            "kappa_1_per_m": kappa,
            "center_error_m": center_error,
            "heading_error_rad": heading_error,
            "polygon_vertex_error_m": polygon_error,
            "support_function_error_m": support_error,
            "samples": constraint.sample_count,
            "scalar_rows": constraint.scalar_row_count,
            "rows_by_vehicle": dict(constraint.rows_by_vehicle),
            "template_id": scene["template"].template_id,
        })
    return rows


def independent_row_metrics():
    scene = _scene(psi0=0.7, kappa=-0.1)
    request = scene["request"]
    config = scene["config"]
    constraint = scene["constraint"]
    packet = _search_packet(request, config)
    vector = _z(-0.1)
    wrapper = _CorridorWrapper(constraint)
    equality_end, signed_end, corridor_end = _constraint_value_offsets(
        request, config, packet, wrapper)
    values = _numeric_constraint_vector(
        vector, request, config, packet,
        int(config.collocation_points), corridor_constraint=wrapper)
    formation_values = values[corridor_end - constraint.scalar_row_count:
                              corridor_end]
    independent = _independent_rows(scene)
    difference = np.abs(formation_values - independent)
    return {
        "psi0_rad": 0.7,
        "kappa_1_per_m": -0.1,
        "equality_end": equality_end,
        "signed_end": signed_end,
        "corridor_end": corridor_end,
        "formation_row_count": int(constraint.scalar_row_count),
        "formation_slice_rows": int(formation_values.size),
        "independent_rows": int(independent.size),
        "max_abs_difference_m": float(np.max(difference)),
    }


def casadi_g_metrics():
    scene = _scene(psi0=0.0, kappa=0.1)
    request = scene["request"]
    config = scene["config"]
    packet = _search_packet(request, config)
    constraint = scene["constraint"]
    wrapper = _CorridorWrapper(constraint)
    vector = _z(0.1)
    equality_end, signed_end, corridor_end = _constraint_value_offsets(
        request, config, packet, wrapper)
    numeric = _numeric_constraint_vector(
        vector, request, config, packet,
        int(config.collocation_points), corridor_constraint=wrapper)
    nlp, built = _casadi_nlp(request, config, packet, wrapper)
    _solver, _lower_x, _upper_x, lower_g, upper_g = built
    g_function = casadi.Function("g", [nlp["x"]], [nlp["g"]])
    symbolic = np.asarray(g_function(vector), dtype=float).ravel()
    formation_slice = slice(corridor_end - constraint.scalar_row_count,
                            corridor_end)
    difference = np.abs(symbolic - numeric)
    return {
        "g_rows": int(symbolic.size),
        "numeric_rows": int(numeric.size),
        "formation_first_g_index": int(
            corridor_end - constraint.scalar_row_count),
        "formation_last_g_index": int(corridor_end - 1),
        "max_abs_numeric_vs_casadi_m": float(np.max(difference)),
        "formation_lower_g_all_neg_inf": bool(
            np.all(np.isneginf(lower_g[formation_slice]))),
        "formation_upper_g_all_zero": bool(
            np.all(upper_g[formation_slice] == 0.0)),
        "formation_finite_rows": int(
            np.count_nonzero(np.isfinite(symbolic[formation_slice]))),
    }


def jacobian_metrics():
    scene = _scene()
    request = scene["request"]
    config = scene["config"]
    packet = _search_packet(request, config)
    constraint = scene["constraint"]
    wrapper = _CorridorWrapper(constraint)
    vector = _z(0.0)
    _equality_end, _signed_end, corridor_end = _constraint_value_offsets(
        request, config, packet, wrapper)
    nlp, _built = _casadi_nlp(request, config, packet, wrapper)
    jacobian = casadi.Function(
        "jacobian", [nlp["x"]], [casadi.jacobian(nlp["g"], nlp["x"])])
    analytic = np.asarray(jacobian(vector), dtype=float)
    formation_slice = slice(
        corridor_end - constraint.scalar_row_count, corridor_end)
    step = 1e-6
    columns = []
    for column in (0, 1, 2, 9, 10, 18):
        plus = vector.copy()
        minus = vector.copy()
        plus[column] += step
        minus[column] -= step
        plus_values = _numeric_constraint_vector(
            plus, request, config, packet,
            int(config.collocation_points),
            corridor_constraint=wrapper)[formation_slice]
        minus_values = _numeric_constraint_vector(
            minus, request, config, packet,
            int(config.collocation_points),
            corridor_constraint=wrapper)[formation_slice]
        finite_difference = (plus_values - minus_values) / (2.0 * step)
        expected = analytic[formation_slice, column]
        absolute = np.abs(finite_difference - expected)
        nonzero = np.abs(expected) > 1e-3
        columns.append({
            "column": column,
            "max_abs_difference": float(np.max(absolute)),
            "max_relative_difference_nonzero": (
                float(np.max(np.abs(
                    (finite_difference[nonzero] - expected[nonzero])
                    / finite_difference[nonzero])))
                if np.any(nonzero) else None),
        })
    return {"finite_difference_step": step, "columns": columns}


def solve_metrics():
    scene = _scene()
    config = replace_config_scope(
        scene["config"], FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
        max_seconds=20.0)
    result = solve_open_path_guided(
        scene["request"], config,
        route_guides=(scene["guide"],),
        corridor_chain=scene["chain"],
        corridor_allocation=scene["allocation"],
        formation_template=scene["template"],
        formation_planning_context_sha256=CONTEXT_SHA)
    return {
        "status": result.status,
        "reason": result.reason,
        "solver_used": result.solver_used,
        "wall_time_s": result.wall_time_s,
        "audit_passed": bool(getattr(result.audit, "passed", False)),
        "formation_corridor_scope": result.formation_corridor_scope,
        "formation_template_id": result.formation_template_id,
        "formation_obb_constraint_count":
            result.formation_obb_constraint_count,
        "corridor_halfspace_constraint_count":
            result.corridor_halfspace_constraint_count,
        "corridor_max_residual_m": result.corridor_max_residual_m,
        "template_id_expected": scene["template"].template_id,
        "compiled_scalar_rows": scene["constraint"].scalar_row_count,
        "path_length_m": getattr(
            getattr(result.path, "summary", None), "length_m", None),
    }


def main():
    metrics = {
        "geometry": geometry_metrics(),
        "independent_rows": independent_row_metrics(),
        "casadi_g": casadi_g_metrics(),
        "jacobian": jacobian_metrics(),
        "solve": solve_metrics(),
    }
    json.dump(metrics, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()