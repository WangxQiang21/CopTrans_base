"""Geometric and connectivity checks for the optional corridor proposal."""
import time

import numpy as np

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import VehicleShape
from fwis_backends.multi_4wis.formation_continuous_a.route_processing import segment_is_free
from fwis_backends.multi_4wis.uniform_convex_corridor import (
    convex_corridor, rectangle_graph, shortcut_corridor,
)
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_backends.multi_4wis.uniform_reachability import compare_uniform_timing
from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig


STEPS = ((1, 0), (-1, 0), (0, 1), (0, -1))


def test_rectangle_partition_and_positive_width_portals():
    blocked = np.ones((6, 6), dtype=bool)
    blocked[:3, :3] = False
    blocked[3:, 2:] = False
    regions, adjacency = rectangle_graph(blocked, lambda: False)
    counts = np.zeros_like(blocked, dtype=int)
    for region in regions:
        counts[region.bottom:region.top, region.left:region.right] += 1
    np.testing.assert_array_equal(counts, ~blocked)
    assert adjacency == [[(1, 2, 3, 3)], [(0, 2, 3, 3)]]

    grid = Grid(blocked, 1., 0., 0.)
    route = convex_corridor(grid, blocked, (.5, .5), (5.5, 5.5), STEPS,
                            deadline=time.perf_counter() + 1, max_expansions=50)
    assert route.status == "FOUND"
    assert route.points[0] == (.5, .5) and route.points[-1] == (5.5, 5.5)
    assert all(segment_is_free(grid, first, second)
               for first, second in zip(route.points, route.points[1:]))


def test_corner_contact_is_disconnected_and_expired_request_stops():
    blocked = np.array([[False, True], [True, False]])
    grid = Grid(blocked, 1., 0., 0.)
    args = (grid, blocked, (.5, .5), (1.5, 1.5), STEPS)
    assert convex_corridor(*args, deadline=time.perf_counter() + 1,
                           max_expansions=10).status == "CORRIDOR_DISCONNECTED"
    assert convex_corridor(*args, deadline=time.perf_counter(),
                           max_expansions=10).status == "CORRIDOR_BUDGET"


def test_shortcut_respects_region_union_direction_and_cancellation():
    blocked = np.ones((6, 6), dtype=bool)
    blocked[:3, :3] = False
    blocked[3:, 2:] = False
    grid = Grid(blocked, 1., 0., 0.)
    start, goal = (.5, .5), (4.5, 5.5)
    route = convex_corridor(grid, blocked, start, goal, STEPS,
                            deadline=time.perf_counter() + 1, max_expansions=50)
    assert len(route.points) > 2
    assert shortcut_corridor(grid, route, lambda *_: True, lambda: False) == (start, goal)
    assert shortcut_corridor(grid, route, lambda *_: False, lambda: False) == route.points
    assert shortcut_corridor(grid, route, lambda *_: True, lambda: True) == route.points

    goal = (5.5, 3.5)
    route = convex_corridor(grid, blocked, start, goal, STEPS,
                            deadline=time.perf_counter() + 1, max_expansions=50)
    points = shortcut_corridor(grid, route, lambda *_: True, lambda: False)
    assert len(points) > 2 and points[0] == start and points[-1] == goal


def test_common_research_timeline_keeps_setup_and_spin_intact():
    grid = Grid(np.zeros((80, 80), dtype=bool), .1, 0., 0.)
    model = FormationModel([VehicleOffset(1, 1., 0.), VehicleOffset(2, 0., 1.),
                            VehicleOffset(3, 0., -1.)])
    request = UniformPlanningRequest(
        grid, model, dict.fromkeys(model.vehicle_ids, VehicleShape(.738, .500)),
        FormationPose(3., 3., 0.), dict.fromkeys(model.vehicle_ids, 0.),
        FormationPose(4.522379369389306, 3.34843514176563, .45),
        search_config=GraphSearchConfig(direct_goal_translation=True, convex_corridor=True))
    result = plan_uniform(request)
    assert result.executable
    digest = result.plan.model_sha256()
    phases = compare_uniform_timing(result.plan)
    assert any(phase.profile is not None for phase in phases)
    assert any(phase.mode == "S" and phase.profile is None for phase in phases)
    time_s = 0.
    for new, old in zip(phases, result.plan.phases, strict=True):
        assert new.t0_s == time_s
        time_s += new.duration_s
        if new.profile is None:
            assert new.duration_s == old.duration_s
        else:
            assert new.profile.progress[0] == 0.
            assert new.profile.progress[-1] == old.edge.control.parameter_length
            assert new.profile.speeds[0] == new.profile.speeds[-1] == 0.
    assert result.plan.model_sha256() == digest
