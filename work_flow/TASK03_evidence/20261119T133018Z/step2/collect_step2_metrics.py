#!/usr/bin/env python3
"""TASK03 Step 2 evidence: measured finite Portal-template behaviour.

The assertions live in ``tests/test_virtual_a_formation_corridor.py``.  This
script records the actual audited scene, candidate counts, cuts, cover
assignments, real appended row counts and the COVER_TEMPLATE_INSUFFICIENT
witness.  It imports the test fixtures so the numbers come from the very same
scene the tests exercise.

Run from the worktree root with the TASK03 plan's PYTHONPATH.
"""
import json
import math
import os
import sys
import traceback

ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    VirtualAContractError,
)
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (  # noqa: E402
    build_formation_corridor_constraint,
    finite_cover_options,
    vehicle_frames,
)
from tests.test_virtual_a_formation_corridor import (  # noqa: E402
    CONTEXT_SHA,
    MARGIN_M,
    PORTAL_THETA,
    _box,
    _build_portal_candidates,
    _point_in_region,
    _portal_scene,
    _world_vertices,
)


def _template_summary(template):
    return {
        "template_id": template.template_id,
        "segments": [
            {
                "sigma_start": float(segment.sigma_start),
                "sigma_end": float(segment.sigma_end),
                "covers": [
                    {
                        "vehicle_id": int(cover.vehicle_id),
                        "decomposition": cover.decomposition,
                        "region_indices": [
                            int(piece.region_index)
                            for piece in cover.pieces],
                    }
                    for cover in segment.covers
                ],
            }
            for segment in template.segments
        ],
    }


def _positive(scene):
    assessment = scene["assessment"]
    candidates = _build_portal_candidates(scene)
    template = candidates.templates[0]
    built = build_formation_corridor_constraint(
        template, scene["request"], scene["chain"], scene["config"],
        allocation=scene["allocation"], guide=scene["guide"],
        planning_context_sha256=CONTEXT_SHA)
    boundary = template.segments[0].sigma_end
    boundary_samples = [
        sample for group in built.groups for sample in group
        if abs(sample.sigma - boundary) <= 1e-12]
    return {
        "chain_audit": {
            "passed": bool(scene["chain_audit"]["passed"]),
            "reason": scene["chain_audit"]["reason"],
        },
        "assessment": {
            "candidate_id": assessment["candidate_id"],
            "map_sample_status": assessment["map_sample_status"],
            "map_sample_counts": assessment["map_sample_counts"],
            "coverage_sample_counts": assessment.get(
                "coverage_sample_counts"),
            "candidate_certification": assessment[
                "candidate_certification"],
            "continuously_certified": assessment[
                "continuously_certified"],
            "formation_obb_hard_constraints": assessment[
                "formation_obb_hard_constraints"],
        },
        "candidate_set": {
            "template_count": len(candidates.templates),
            "template_ids": [
                item.template_id for item in candidates.templates],
            "cuts": [float(value) for value in candidates.cuts],
            "station_count": int(candidates.station_count),
            "synthetic_station_count": int(
                candidates.synthetic_station_count),
            "per_segment_vehicle_option_counts": [
                [int(value) for value in row]
                for row in candidates.per_segment_vehicle_option_counts],
            "truncated_option_count": int(
                candidates.truncated_option_count),
        },
        "first_template": _template_summary(template),
        "first_template_rows": {
            "sample_count": int(built.sample_count),
            "scalar_row_count": int(built.scalar_row_count),
            "rows_by_vehicle": [
                [int(vehicle_id), int(count)]
                for vehicle_id, count in built.rows_by_vehicle],
            "segment_row_counts": [
                int(count) for count in built.segment_row_counts],
            "vehicle_count": len(vehicle_frames(scene["request"])),
        },
        "first_internal_cut_double_sided": {
            "sigma": float(boundary),
            "sample_count": len(boundary_samples),
            "segment_indices": sorted({
                int(sample.template_segment_index)
                for sample in boundary_samples}),
            "sources": sorted({
                sample.source for sample in boundary_samples}),
            "body_regions": sorted({
                int(sample.body_region_index)
                for sample in boundary_samples}),
            "covers_match_samples": all(
                sample.decomposition == next(
                    cover.decomposition
                    for cover in template.segments[
                        sample.template_segment_index].covers
                    if cover.vehicle_id == sample.vehicle_id)
                for sample in boundary_samples),
        },
        "dialect": "row counts are actual appended scalar inequalities only",
    }


def _insufficient():
    scene = _portal_scene(overlap_half=0.2)
    assessment = scene["assessment"]
    try:
        _build_portal_candidates(scene)
    except VirtualAContractError as exc:
        failure = {
            "reason": exc.reason,
            "detail": str(exc),
            "is_physical_infeasibility": False,
        }
    else:
        failure = {"reason": None, "detail": "unexpectedly built"}
    return {
        "chain_audit": {
            "passed": bool(scene["chain_audit"]["passed"]),
            "reason": scene["chain_audit"]["reason"],
        },
        "assessment": {
            "map_sample_status": assessment["map_sample_status"],
            "map_sample_counts": assessment["map_sample_counts"],
            "candidate_certification": assessment[
                "candidate_certification"],
        },
        "builder_failure": failure,
        "note": (
            "map samples are clear; finite whole/diagonal templates cannot "
            "express the union-covered pose, so the candidate is unknown, "
            "not physically infeasible"),
    }


def _direct():
    frame = vehicle_frames(_portal_scene()["request"])[0]
    alpha = math.atan2(frame.width_m, frame.length_m)
    upper = _box(-0.30, 0.32, 0.30, 0.45, (0.0, 0.32), (0.0, 0.45))
    lower = _box(-0.30, -0.45, 0.30, -0.32, (0.0, -0.45), (0.0, -0.32))
    left = _box(-10.0, -10.0, 0.0, 10.0, (-1.0, 0.0), (0.0, 0.0))
    right = _box(0.0, -10.0, 10.0, 10.0, (0.0, 0.0), (1.0, 0.0))
    corners = _world_vertices(
        frame.local_corners, (0.0, 0.0), math.pi / 2.0)
    options = finite_cover_options(
        frame, (0.0, 0.0), math.pi / 2.0, (upper, lower), (0, 1))
    return {
        "e07_nonconvex_gap": {
            "frame": {
                "vehicle_id": int(frame.vehicle_id),
                "length_m": float(frame.length_m),
                "width_m": float(frame.width_m)},
            "all_four_corners_in_union": all(
                _point_in_region(corner, upper) or
                _point_in_region(corner, lower) for corner in corners),
            "center_in_union": bool(
                _point_in_region((0.0, 0.0), upper) or
                _point_in_region((0.0, 0.0), lower)),
            "finite_options": [
                {"decomposition": option.decomposition,
                 "region_indices": list(option.region_indices)}
                for option in options],
        },
        "diagonal_zero": [
            {"decomposition": option.decomposition,
             "region_indices": list(option.region_indices)}
            for option in finite_cover_options(
                frame, (0.0, 0.0), math.pi / 2.0 - alpha,
                (left, right), (0, 1))],
        "diagonal_one": [
            {"decomposition": option.decomposition,
             "region_indices": list(option.region_indices)}
            for option in finite_cover_options(
                frame, (0.0, 0.0), math.pi / 2.0 + alpha,
                (left, right), (0, 1))],
        "long_axis_crossing": [
            {"decomposition": option.decomposition,
             "region_indices": list(option.region_indices)}
            for option in finite_cover_options(
                frame, (0.0, 0.0), 0.0, (left, right), (0, 1))],
        "portal_theta_rad": PORTAL_THETA,
        "rectangle_half_diagonal_angle_rad": alpha,
    }


def main():
    payload = {
        "role": "task03_step2_finite_portal_templates_measured",
        "worktree": ROOT,
        "positive_scene": _positive(_portal_scene(overlap_half=0.3)),
        "insufficient_scene": _insufficient(),
        "direct_cover_geometry": _direct(),
    }
    json.dump(payload, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        traceback.print_exc()
        raise