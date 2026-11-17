"""Collect the C1 corridor evidence sample and measured summary.

Writes ``sample_chain.json`` (grid/route/A,b/vertices/portal/SAT for V1) and
``measurements.json`` (category results, explicit rejections, 50x50 timing).
This script is evidence, not production code.
"""
import json
import math
import platform
import sys
import time

import numpy as np
import scipy

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    ConvexCorridor,
    CorridorChain,
    GuideRoute,
    VirtualAContractError,
    grid_signature,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    audit_corridor_chain,
    build_corridor_chain,
)


def make_grid(occupied, resolution=0.05, origin_x=0.0, origin_y=0.0):
    return Grid(np.asarray(occupied, dtype=bool), resolution=resolution,
                origin_x=origin_x, origin_y=origin_y)


def make_route(grid, points, source="c1_evidence"):
    return GuideRoute(
        points=tuple(points), source=source, grid_sha256=grid_signature(grid))


def summarize_chain(grid, points, source="c1_evidence"):
    route = make_route(grid, points, source=source)
    chain = build_corridor_chain(grid, route)
    audit = audit_corridor_chain(grid, chain)
    return route, chain, audit


def compact_result(name, grid, route, chain, audit):
    return {
        "name": name,
        "route": route.as_dict(),
        "chain_digest": chain.digest,
        "region_count": len(chain.regions),
        "passed": audit["passed"],
        "reason": audit["reason"],
        "min_region_area_m2": audit["min_region_area_m2"],
        "max_seed_violation_m": audit["max_seed_violation_m"],
        "min_portal_radius_m": audit["min_portal_radius_m"],
        "min_portal_area_m2": audit["min_portal_area_m2"],
        "first_collision": audit["first_collision"],
        "sat_checked_cells": [
            region["sat_checked_cells"] for region in audit["regions"]],
        "regions": [
            {"seed_start": region["seed_start"],
             "seed_end": region["seed_end"],
             "area_m2": region["area_m2"]}
            for region in audit["regions"]],
        "portals": audit["portals"],
    }


def square_corridor(x0, y0, x1, y1, seed_start, seed_end):
    return ConvexCorridor(
        seed_start=seed_start, seed_end=seed_end,
        A=((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)),
        b=(-x0, x1, -y0, y1),
        vertices=((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def main(outdir):
    measurements = {
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
        },
        "categories": {},
        "explicit_rejections": {},
    }

    # 1. Single box: the sample handed to V1, with the hand-checked plane.
    occupied = np.zeros((80, 80), dtype=bool)
    occupied[20:40, 20:40] = True
    grid = make_grid(occupied)
    route, chain, audit = summarize_chain(
        grid, [(1.0, 3.0), (3.0, 3.0), (3.0, 0.5)], source="single_box")
    expected_b = (-3.0 + 0.05 - 2.0) / 2.0 - 1e-8
    found = [
        [normal[0], normal[1], limit]
        for normal, limit in zip(
            chain.regions[0].A, chain.regions[0].b)
        if abs(normal[0]) < 1e-12 and abs(normal[1] + 1.0) < 1e-12
        and abs(limit - expected_b) < 1e-12]
    sample = {
        "stage": "C1",
        "case": "single_box_hand_plane",
        "grid": {
            "height": grid.height,
            "width": grid.width,
            "resolution": grid.res,
            "origin_x": grid.ox,
            "origin_y": grid.oy,
            "grid_sha256": grid_signature(grid),
            "occupied_cells": [
                [int(iy), int(ix)]
                for iy, ix in zip(*np.nonzero(occupied))],
        },
        "route": route.as_dict(),
        "chain": chain.as_dict(),
        "audit": audit,
        "hand_check": {
            "cell": "top row of the box",
            "normal": [0.0, -1.0],
            "h": -3.0,
            "l": -2.0,
            "rho": 0.05,
            "b_expected": expected_b,
            "matching_A_b_rows": found,
            "region0_min_y": min(y for _x, y in chain.regions[0].vertices),
            "region1_min_x": min(x for x, _y in chain.regions[1].vertices),
        },
        "notes": {
            "is_executable_path": False,
            "accepts_three_vehicle_safety_claim": False,
            "portal_and_sat_source": "audit.portals and audit.regions",
        },
    }
    with open(outdir + "/sample_chain.json", "w") as handle:
        json.dump(sample, handle, indent=2, sort_keys=True)
        handle.write("\n")
    measurements["categories"]["single_box"] = compact_result(
        "single_box", grid, route, chain, audit)
    measurements["categories"]["single_box"]["hand_b_expected"] = expected_b
    measurements["categories"]["single_box"]["hand_b_found"] = found

    # 2. Empty field with three seeds.
    grid = make_grid(np.zeros((100, 100), dtype=bool))
    route, chain, audit = summarize_chain(grid, [
        (0.5, 0.5), (2.0, 0.5), (2.0, 2.5), (4.0, 2.5)], source="empty")
    measurements["categories"]["empty"] = compact_result(
        "empty", grid, route, chain, audit)

    # 3. L corner.
    occupied = np.zeros((100, 100), dtype=bool)
    occupied[10:60, 20:24] = True
    grid = make_grid(occupied)
    route, chain, audit = summarize_chain(grid, [
        (0.5, 0.25), (0.5, 3.5), (3.5, 3.5), (3.5, 0.25)], source="l_corner")
    measurements["categories"]["l_corner"] = compact_result(
        "l_corner", grid, route, chain, audit)

    # 4. Concave U.
    occupied = np.zeros((120, 120), dtype=bool)
    occupied[10:60, 20:24] = True
    occupied[10:60, 60:64] = True
    occupied[10:14, 20:64] = True
    grid = make_grid(occupied)
    route, chain, audit = summarize_chain(grid, [
        (2.1, 1.5), (2.1, 3.5), (4.0, 3.5)], source="concave_u")
    measurements["categories"]["concave_u"] = compact_result(
        "concave_u", grid, route, chain, audit)

    # 5. Narrow gap.
    occupied = np.zeros((100, 100), dtype=bool)
    occupied[0:38, :] = True
    occupied[44:100, :] = True
    grid = make_grid(occupied)
    route, chain, audit = summarize_chain(grid, [
        (0.5, 2.05), (2.5, 2.05), (4.5, 2.05)], source="narrow_gap")
    measurements["categories"]["narrow_gap"] = compact_result(
        "narrow_gap", grid, route, chain, audit)

    # 6. Map boundary route.
    grid = make_grid(np.zeros((80, 80), dtype=bool))
    route, chain, audit = summarize_chain(grid, [
        (0.025, 0.025), (3.975, 0.025),
        (3.975, 3.975), (0.025, 3.975)], source="map_boundary")
    measurements["categories"]["map_boundary"] = compact_result(
        "map_boundary", grid, route, chain, audit)

    # 7. Obstacle at the far edge of the padding box.
    occupied = np.zeros((100, 100), dtype=bool)
    occupied[40, 78] = True
    grid = make_grid(occupied)
    route, chain, audit = summarize_chain(grid, [
        (0.5, 2.0), (2.0, 2.0)], source="far_padding_obstacle")
    measurements["categories"]["far_padding_obstacle"] = compact_result(
        "far_padding_obstacle", grid, route, chain, audit)
    measurements["categories"]["far_padding_obstacle"][
        "max_x_expected"] = 2.975 - 1e-8
    measurements["categories"]["far_padding_obstacle"][
        "max_x_found"] = max(
            x for x, _y in chain.regions[0].vertices)

    # 8. Point-touch portal: hand-built valid regions, audit must reject.
    grid = make_grid(np.zeros((80, 80), dtype=bool))
    first = square_corridor(
        0.5, 0.5, 1.5, 1.5, (0.75, 0.75), (1.5, 1.5))
    second = square_corridor(
        1.5, 1.5, 2.5, 2.5, (1.5, 1.5), (2.25, 2.25))
    chain = CorridorChain(
        (first, second), "point_touch", grid_signature(grid))
    audit = audit_corridor_chain(grid, chain)
    measurements["explicit_rejections"]["point_touch_portal"] = {
        "passed": audit["passed"],
        "reason": audit["reason"],
        "detail": audit["detail"],
        "min_portal_radius_m": audit["min_portal_radius_m"],
    }

    # 9. Hand-built corridor covering an occupied cell: independent SAT.
    occupied = np.zeros((80, 80), dtype=bool)
    occupied[20, 20] = True
    grid = make_grid(occupied)
    corridor = square_corridor(
        0.9, 0.9, 1.2, 1.2, (0.95, 0.95), (1.15, 1.15))
    chain = CorridorChain((corridor,), "sat_negative", grid_signature(grid))
    audit = audit_corridor_chain(grid, chain)
    measurements["explicit_rejections"]["sat_cell_overlap"] = {
        "passed": audit["passed"],
        "reason": audit["reason"],
        "detail": audit["detail"],
        "first_collision": audit["first_collision"],
    }

    # 10. Seed touching an obstacle: build must reject explicitly.
    grid = make_grid(occupied)
    route = make_route(grid, [(0.5, 0.5), (1.0, 1.0)], source="touch_seed")
    try:
        build_corridor_chain(grid, route)
        measurements["explicit_rejections"]["seed_touching_obstacle"] = {
            "raised": False}
    except VirtualAContractError as error:
        measurements["explicit_rejections"]["seed_touching_obstacle"] = {
            "raised": True,
            "reason": error.reason,
            "detail": error.detail,
        }

    # 11. Stored halfspaces looser than the audited polygon: C2 consumes A,b,
    # so the audit must reject a certificate whose halfspaces cover an
    # obstacle outside the polygon.
    occupied = np.zeros((5, 5), dtype=bool)
    occupied[0, 0] = True
    grid = make_grid(occupied, resolution=1.0)
    loose = ConvexCorridor(
        seed_start=(3.2, 3.2), seed_end=(3.8, 3.2),
        A=((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)),
        b=(0.0, 4.0, 0.0, 4.0),
        vertices=((3.0, 3.0), (4.0, 3.0), (4.0, 4.0), (3.0, 4.0)))
    chain = CorridorChain((loose,), "loose_halfspaces", grid_signature(grid))
    audit = audit_corridor_chain(grid, chain)
    measurements["explicit_rejections"]["loose_halfspaces"] = {
        "passed": audit["passed"],
        "reason": audit["reason"],
        "detail": audit["detail"],
    }

    # 12. Vertex 5e-10 m outside the map must be rejected: no tolerance slack
    # on the closed-map containment check.
    grid = make_grid(np.zeros((20, 20), dtype=bool), resolution=0.1)
    corridor = square_corridor(
        -5e-10, 0.5, 0.5, 1.5, (0.1, 0.75), (0.4, 1.25))
    chain = CorridorChain(
        (corridor,), "subnanometre_outside", grid_signature(grid))
    audit = audit_corridor_chain(grid, chain)
    measurements["explicit_rejections"]["vertex_just_outside_map"] = {
        "passed": audit["passed"],
        "reason": audit["reason"],
        "detail": audit["detail"],
    }
    # 13. 50x50 sample timing (no performance promise).
    occupied = np.zeros((50, 50), dtype=bool)
    occupied[0:41, 10] = True
    occupied[9:50, 20] = True
    occupied[0:41, 30] = True
    occupied[9:50, 40] = True
    grid = make_grid(occupied, resolution=0.1)
    route = make_route(grid, [
        (0.5, 4.5), (1.5, 4.5), (1.5, 0.5), (2.5, 0.5),
        (2.5, 4.5), (3.5, 4.5), (3.5, 0.5), (4.5, 0.5)],
        source="timing_50x50")
    started = time.perf_counter()
    chain = build_corridor_chain(grid, route)
    built = time.perf_counter()
    audit = audit_corridor_chain(grid, chain)
    audited = time.perf_counter()
    measurements["fifty_by_fifty"] = {
        "grid_shape": [grid.height, grid.width],
        "resolution": grid.res,
        "route_points": len(route.points),
        "region_count": len(chain.regions),
        "build_seconds": built - started,
        "audit_seconds": audited - built,
        "total_seconds": audited - started,
        "passed": audit["passed"],
        "reason": audit["reason"],
        "note": "one sample on this machine; no performance promise",
    }

    with open(outdir + "/measurements.json", "w") as handle:
        json.dump(measurements, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print(json.dumps({
        "sample_passed": audit["passed"],
        "categories": {
            name: (value["passed"], value["reason"])
            for name, value in measurements["categories"].items()},
        "rejections": measurements["explicit_rejections"],
        "timing": measurements["fifty_by_fifty"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: collect_c1_evidence.py <evidence-dir>")
    main(sys.argv[1])