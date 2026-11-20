#!/usr/bin/env python3
"""TASK03 rework 02 Step 1/2 diagnostic scan.

Runs the FULL production ``plan_virtual_a`` on deterministic synthetic
occupancy grids (no route/guide/template mocks).  Wrappers around
``certify_formation_path`` / ``_formation_sample_audit`` only OBSERVE the
candidate paths and certificates; they never change the returned objects.

One JSON summary per scene is printed on stdout (prefix ``SUMMARY ``).
"""
import argparse
import dataclasses
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, '/home/wq/whut_thesis/expv3_multi_virtual_a/tests')
sys.path.insert(
    0, '/home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/src/fwis_v3_optimizer/src')

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    grid_signature, research_planner_config)
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a import pipeline as pipeline_module
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    audit_path_in_corridors)
from virtual_a_g4_support import make_request, map_request


def build_grid(a, b, deg, wall_x, wall_y0, wall_y1, wall_half=0.1, res=0.1):
    theta = math.radians(deg)
    gx, gy = a + b * math.cos(theta), b * math.sin(theta)
    xmax = max(gx, wall_x) + 2.5
    ymax = max(gy, wall_y1) + 2.5
    ymin = min(0.0, wall_y0) - 2.5
    cols = int((xmax + 2.5) / res) + 4
    rows = int((ymax - ymin + 2.5) / res) + 4
    grid = Grid(np.zeros((rows, cols), dtype=bool), res, -2.5, ymin - 2.5,
                length=0.738, width=0.500)
    ix0 = int((wall_x - wall_half - grid.ox) / res)
    ix1 = int((wall_x + wall_half - grid.ox) / res)
    iy0 = int((wall_y0 - grid.oy) / res)
    iy1 = int((wall_y1 - grid.oy) / res)
    grid.occ[iy0:iy1 + 1, ix0:ix1 + 1] = True
    return grid, (gx, gy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--a', type=float, required=True)
    ap.add_argument('--b', type=float, required=True)
    ap.add_argument('--deg', type=float, required=True)
    ap.add_argument('--pad', type=float, default=3.0)
    ap.add_argument('--wall-x', type=float, required=True)
    ap.add_argument('--wall-y0', type=float, default=1.0)
    ap.add_argument('--wall-y1', type=float, default=12.0)
    ap.add_argument('--wall-half', type=float, default=0.1)
    ap.add_argument('--psi0', type=float, default=0.0)
    ap.add_argument('--label', default='scene')
    ap.add_argument('--seconds', type=float, default=60.0)
    args = ap.parse_args()

    grid, (gx, gy) = build_grid(
        args.a, args.b, args.deg, args.wall_x, args.wall_y0, args.wall_y1,
        wall_half=args.wall_half)
    start = FormationPose(0.0, 0.0, float(args.psi0))
    goal = FormationPose(gx, gy, 0.0)
    planner = research_planner_config(
        guidance_mode='corridor', corridor_enforcement='hard',
        formation_corridor_scope=FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
        corridor_box_padding_m=args.pad, max_initials=6,
        max_seconds=args.seconds)
    request = make_request(
        start, goal,
        start_headings={1: args.psi0, 2: args.psi0, 3: args.psi0})

    captured = []
    assessments = []
    route_sets = []
    original_certify = optimizer_module.certify_formation_path
    original_assess = pipeline_module.assess_formation_guide
    original_routes = pipeline_module.make_route_guides

    def wrapped(path, req, chain, template, **kwargs):
        cert = original_certify(path, req, chain, template, **kwargs)
        captured.append((path, cert, template, kwargs.get('allocation'), chain))
        return cert

    def wrapped_assess(*args, **kwargs):
        assessment = original_assess(*args, **kwargs)
        assessments.append(assessment)
        return assessment

    def wrapped_routes(*args, **kwargs):
        routes = original_routes(*args, **kwargs)
        route_sets.append([{
            'source': route.source,
            'grid_sha256': route.grid_sha256,
            'length_m': float(route.length_m),
            'points': [list(point) for point in route.points],
        } for route in routes])
        return routes

    optimizer_module.certify_formation_path = wrapped
    pipeline_module.assess_formation_guide = wrapped_assess
    pipeline_module.make_route_guides = wrapped_routes
    try:
        result = pipeline_module.plan_virtual_a(
            map_request(request, grid, planner=planner))
    finally:
        optimizer_module.certify_formation_path = original_certify
        pipeline_module.assess_formation_guide = original_assess
        pipeline_module.make_route_guides = original_routes

    output_dir = os.path.dirname(os.path.abspath(__file__))
    for name, payload in (('assessments', assessments),
                          ('route_sets', route_sets)):
        output_path = os.path.join(
            output_dir, '{}.{}.json'.format(args.label, name))
        with open(output_path, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write('\n')

    collision_stations = []
    for assessment in assessments:
        for station in assessment.get('station_coverage') or []:
            if station.get('map_status') == 'COLLISION_OBSERVED':
                collision_stations.append({
                    'sigma': station.get('sigma'),
                    's_m': station.get('s_m'),
                    'guide_pose': station.get('guide_pose'),
                    'vehicles': station.get('vehicles'),
                })

    audit = result.formation_corridor_audit or {}
    search = result.search or {}
    corridor_rounds = (search.get('corridor') or {}).get('rounds') or []
    chain_info = {}
    allocation = []
    guide_points = []
    if corridor_rounds:
        last = corridor_rounds[-1]
        chain_payload = last.get('chain') or {}
        chain_info = {
            'regions': len(chain_payload.get('regions') or []),
            'digest': last.get('chain_digest'),
        }
        allocation = [list(x) for x in (last.get('allocation') or [])]
        guide_points = [list(p) for p in
                        ((last.get('guide') or {}).get('points') or [])]
        regions_payload = chain_payload.get('regions') or []
        chain_info['region_bounds'] = [
            {'A': r.get('A'), 'b': r.get('b')} for r in regions_payload]

    records = []
    for path, cert, template, alloc, chain in captured:
        record = {
            'template_id': template.template_id,
            'segments': [[float(s.sigma_start), float(s.sigma_end),
                          int(s.f_region_index)] for s in template.segments],
            'certificate_status': cert.get('status'),
            'certificate_reason': cert.get('reason'),
            'path_length_m': float(path.length_m),
        }
        if alloc:
            old = audit_path_in_corridors(
                path, chain, alloc,
                position_budget_m=float(
                    planner.corridor_audit_position_margin_m),
                max_depth=int(planner.corridor_audit_max_depth),
                max_intervals=int(planner.corridor_audit_max_intervals),
                min_width_m=float(planner.corridor_audit_min_width_m))
            failure = old.get('first_failure') or {}
            record['old_audit'] = {
                'passed': old.get('passed'),
                'reason': old.get('reason'),
                'first_failure': {
                    k: failure.get(k) for k in (
                        's_m', 'sigma', 'corridor_index', 'reason',
                        'point_residual_m', 'bound_residual_m', 'width_m')}
                if failure else None,
            }
        records.append(record)

    summary = {
        'label': args.label,
        'params': vars(args),
        'grid_sha256': grid_signature(grid),
        'request_sha256': result.request_sha256,
        'context_sha256': result.planning_context_sha256,
        'goal': [gx, gy],
        'status': result.status,
        'reason': result.reason,
        'executable': bool(result.executable),
        'f_only_gate_migrated': audit.get('f_only_gate_migrated'),
        'chain': chain_info,
        'allocation': allocation,
        'guide_points': guide_points,
        'assessment_count': len(assessments),
        'route_set_count': len(route_sets),
        'collision_stations': collision_stations,
        'audit_status': audit.get('status'),
        'audit_detail': audit.get('detail'),
        'template_attempts': audit.get('template_attempts'),
        'certificate_status': (audit.get('certificate') or {}).get('status'),
        'captures': records,
    }
    print('SUMMARY ' + json.dumps(summary, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
