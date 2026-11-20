#!/usr/bin/env python3
"""Step B/C probe: actual NLP ``g`` total for the union pipeline scene.

Runs the same deterministic straight union scene as
``G4PipelineTests::test_union_scope_certifies_the_f_obb_path_end_to_end``
and wraps ``optimizer._constraint_value_offsets`` (observation only, the
original function is always called) to record the row slice sizes the
solver actually used.  The last recorded call belongs to the accepted
attempt.  Prints JSON.
"""
import json
from unittest import mock

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    research_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a
from tests.virtual_a_g4_support import empty_grid, make_request, map_request


def main():
    records = []
    original = optimizer_module._constraint_value_offsets

    def spy(request, config, packet, corridor_constraint):
        equality_end, signed_end, corridor_end = original(
            request, config, packet, corridor_constraint)
        formation = getattr(corridor_constraint, "formation", None)
        records.append({
            "intervals": int(packet["intervals"]),
            "equality_end": int(equality_end),
            "signed_end": int(signed_end),
            "corridor_end": int(corridor_end),
            "f_rows": (
                None if formation is None
                else int(formation.f_scalar_row_count)),
            "obb_rows": (
                None if formation is None
                else int(formation.scalar_row_count)),
        })
        return equality_end, signed_end, corridor_end

    grid = empty_grid()
    request = make_request(
        FormationPose(0.0, 0.0, 0.0), FormationPose(6.0, 0.0, 0.0))
    planner = research_planner_config(
        guidance_mode="corridor", corridor_enforcement="hard",
        formation_corridor_scope=
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
        max_initials=3, max_seconds=20.0)
    with mock.patch.object(
            optimizer_module, "_constraint_value_offsets",
            side_effect=spy):
        result = plan_virtual_a(map_request(request, grid, planner=planner))

    accepted = [r for r in records if r["obb_rows"]]
    distinct = []
    seen = set()
    for record in records:
        key = json.dumps(record, sort_keys=True)
        if key not in seen:
            seen.add(key)
            distinct.append(record)
    payload = {
        "schema": "task03_rework01.union_g_total_probe.v1",
        "status": result.status,
        "executable": bool(result.executable),
        "formation_f_constraint_count":
            result.formation_f_constraint_count,
        "formation_obb_constraint_count":
            result.formation_obb_constraint_count,
        "offset_calls": len(records),
        "distinct_slices": distinct,
        "accepted_attempt_slice": accepted[-1] if accepted else None,
        "g_total_last_call": records[-1]["corridor_end"] if records else None,
        "attempts": [
            {"index": item.attempt_index,
             "solver": item.solver,
             "solver_status": item.solver_status,
             "status": item.status}
            for item in result.attempts],
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())