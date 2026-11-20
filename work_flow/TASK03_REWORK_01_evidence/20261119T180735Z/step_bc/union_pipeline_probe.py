#!/usr/bin/env python3
"""Step B/C probe: union pipeline row counts and identities.

Runs the same deterministic empty-grid straight union scene as
``tests/test_virtual_a_pipeline.py::
G4PipelineTests::test_union_scope_certifies_the_f_obb_path_end_to_end``
through the production ``plan_virtual_a`` and prints the measured row
counts, certificate and binding identities as JSON.  Read-only probe:
no production code, thresholds or budgets are changed.
"""
import json

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    research_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    corridor_chain_from_payload,
)
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (
    formation_template_from_payload,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a
from tests.virtual_a_g4_support import empty_grid, make_request, map_request


def main():
    grid = empty_grid()
    request = make_request(
        FormationPose(0.0, 0.0, 0.0), FormationPose(6.0, 0.0, 0.0))
    planner = research_planner_config(
        guidance_mode="corridor", corridor_enforcement="hard",
        formation_corridor_scope=
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
        max_initials=3, max_seconds=20.0)
    result = plan_virtual_a(map_request(request, grid, planner=planner))
    audit = result.formation_corridor_audit
    template = formation_template_from_payload(audit["template"])
    chain = corridor_chain_from_payload(
        result.search["corridor"]["rounds"][-1]["chain"])
    certificate = audit["certificate"]
    f_count = result.formation_f_constraint_count
    obb_count = result.formation_obb_constraint_count
    payload = {
        "schema": "task03_rework01.union_pipeline_probe.v1",
        "status": result.status,
        "executable": bool(result.executable),
        "formation_corridor_scope": result.formation_corridor_scope,
        "formation_f_constraint_count": f_count,
        "formation_obb_constraint_count": obb_count,
        "total_halfspace_rows": (
            None if f_count is None
            else int(f_count) + int(obb_count)),
        "template_id": template.template_id,
        "template_schema": audit["template"]["schema_version"],
        "template_compiler": audit["template"]["compiler_version"],
        "segments": [
            {"sigma_start": segment.sigma_start,
             "sigma_end": segment.sigma_end,
             "f_region_index": segment.f_region_index}
            for segment in template.segments],
        "allocation": [list(entry) for entry in audit["allocation"]],
        "chain_regions": len(chain.regions),
        "chain_digest": audit["chain_digest"],
        "certificate_status": certificate["status"],
        "certificate_sha256": certificate["certificate_sha256"],
        "certificate_max_outward_residual_m":
            certificate.get("max_outward_residual_m"),
        "certificate_unclosed": certificate.get("unclosed_interval_count"),
        "certificate_intervals_verified":
            certificate.get("intervals_verified"),
        "certificate_intervals_checked":
            certificate.get("intervals_checked"),
        "identity": certificate["identity"],
        "request_sha256": result.request_sha256,
        "grid_sha256": result.grid_sha256,
        "planning_context_sha256": result.planning_context_sha256,
        "path_length_m": certificate["path_length_m"],
        "path_sha256": certificate["path_sha256"],
        "f_only_gate_migrated": audit["f_only_gate_migrated"],
        "f_only_containment_passed": audit["f_only_containment"]["passed"],
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())