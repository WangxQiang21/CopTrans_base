#!/usr/bin/env python3
"""E04 split evidence probe: offline OBB NLP negative.

Reproduces the scene of
``tests/test_virtual_a_formation_corridor.py::
FormationSampleAuditStep3Tests::test_e04_offline_obb_rows_are_real_and_reject_f_only_feasible``
through the test module's own helpers and prints the measured numbers as
JSON.  Read-only: no production file is modified, no threshold is changed.
"""
import json

import numpy as np

from tests import test_virtual_a_formation_corridor as T
from fwis_v3_optimizer.formation_continuous_a.optimizer import (
    solve_open_path_guided,
)


def main():
    scene = T._step3_scene(y_half=1.1, max_seconds=3.0)
    request = scene["request"]
    config = scene["config"]
    packet = T._search_packet(request, config)
    constraint = T._build_corridor_constraint(
        scene["chain"], scene["allocation"], request, config,
        formation_template=scene["template"],
        planning_context_sha256=T.CONTEXT_SHA,
        formation_guide=scene["guide"])
    formation = constraint.formation
    _equality_end, signed_end, corridor_end = T._constraint_value_offsets(
        request, config, packet, constraint)
    values = T._numeric_constraint_vector(
        T._z(0.0), request, config, packet, int(config.collocation_points),
        corridor_constraint=constraint)
    formation_count = int(formation.scalar_row_count)
    f_rows = values[signed_end:corridor_end - formation_count]
    obb_rows = values[corridor_end - formation_count:corridor_end]
    result = solve_open_path_guided(
        request, config,
        route_guides=(scene["guide"],),
        corridor_chain=scene["chain"],
        corridor_allocation=scene["allocation"],
        formation_template=scene["template"],
        formation_planning_context_sha256=T.CONTEXT_SHA)
    payload = {
        "schema": "task03_rework01.e04_offline_probe.v1",
        "scene": {
            "name": "step3_short_y_half_1.1",
            "guide_sha256": T._sha256_points(scene["guide"]) if hasattr(
                T, "_sha256_points") else None,
            "path_length_ratio_hint": "same scene as the Step 3 audit test",
        },
        "constraint": {
            "sample_count": int(constraint.sample_count),
            "f_scalar_row_count": int(formation.f_scalar_row_count),
            "formation_obb_constraint_count":
                int(constraint.formation_obb_constraint_count),
            "rows_by_vehicle": {str(k): int(v) for k, v in
                                formation.rows_by_vehicle},
            "f_region_by_segment": [int(v) for v in
                                    formation.f_region_by_segment],
            "segment_f_row_counts": [int(v) for v in
                                     formation.segment_f_row_counts],
        },
        "numeric_probe": {
            "vector_size": int(values.size),
            "corridor_end_expected": int(corridor_end),
            "max_f_reference_row": float(np.max(f_rows)),
            "max_obb_row": float(np.max(obb_rows)),
            "obb_row_count": int(obb_rows.size),
        },
        "solve": {
            "status": result.status,
            "reason": result.reason,
            "path_is_none": result.path is None,
            "formation_obb_constraint_count":
                result.formation_obb_constraint_count,
            "formation_obb_max_residual_m":
                result.formation_obb_max_residual_m,
            "attempts": [
                {"index": index, "status": attempt.status,
                 "solver": getattr(attempt, "solver", None),
                 "reason": getattr(attempt, "reason", None)}
                for index, attempt in enumerate(result.attempts)],
            "accepted_attempt": any(
                attempt.status == "SOLVER_SUCCESS_AUDITED"
                for attempt in result.attempts),
        },
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())