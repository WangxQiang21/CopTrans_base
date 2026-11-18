#!/usr/bin/env python3
"""Reproduce the C2-A sample guides and sigma allocations from exact boxes."""
import json
import math

from fwis_v3_optimizer.formation_continuous_a.contracts import (
    ConvexCorridor,
    CorridorChain,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    allocate_corridors,
    make_corridor_guide,
)


GRID = "0" * 64


def box(x0, y0, x1, y1, seed_start, seed_end):
    return ConvexCorridor(
        seed_start=seed_start,
        seed_end=seed_end,
        A=((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)),
        b=(-x0, x1, -y0, y1),
        vertices=((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
    )


def segments_chain(seeds, padding=0.6):
    regions = []
    for start, end in zip(seeds, seeds[1:]):
        regions.append(box(
            min(start[0], end[0]) - padding,
            min(start[1], end[1]) - padding,
            max(start[0], end[0]) + padding,
            max(start[1], end[1]) + padding,
            start,
            end,
        ))
    return CorridorChain(
        regions=tuple(regions), route_source="c2a_sample", grid_sha256=GRID)


def parallel_chain(count):
    return segments_chain(
        [(float(index), 0.0) for index in range(count + 1)])


def case(name, chain, start, goal):
    guide = make_corridor_guide(chain, start, goal)
    allocation = allocate_corridors(chain, guide, 8)
    return {
        "name": name,
        "region_count": len(chain.regions),
        "start": [start[0], start[1]],
        "goal": [goal[0], goal[1]],
        "guide_points": [[x, y] for x, y in guide.points],
        "guide_source": guide.source,
        "guide_quality": guide.quality,
        "allocation_intervals": 8,
        "allocation": [
            [left, right, corridor] for left, right, corridor in allocation],
        "allocation_count": len(allocation),
        "coverage": [allocation[0][0], allocation[-1][1]],
        "corridor_indices": sorted({item[2] for item in allocation}),
        "contiguous": all(
            math.isclose(first[1], second[0], abs_tol=1e-15)
            for first, second in zip(allocation, allocation[1:])),
    }


def u_chain():
    return CorridorChain(
        regions=(
            box(-0.5, -0.5, 2.5, 0.5, (0.0, 0.0), (2.0, 0.0)),
            box(1.5, -0.5, 2.5, 2.5, (2.0, 0.0), (2.0, 2.0)),
            box(-0.5, 1.5, 2.5, 2.5, (2.0, 2.0), (0.0, 2.0)),
        ),
        route_source="c2a_u_sample",
        grid_sha256=GRID,
    )


print(json.dumps({
    "cases": [
        case("M1", parallel_chain(1), (0.0, 0.0), (1.0, 0.0)),
        case(
            "M3_unequal",
            segments_chain(
                [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (6.0, 0.0)],
                padding=0.4),
            (0.0, 0.0), (6.0, 0.0)),
        case("M12", parallel_chain(12), (0.0, 0.0), (12.0, 0.0)),
        case("U_topology", u_chain(), (0.0, 0.0), (0.0, 2.0)),
    ],
}, indent=2, sort_keys=True))