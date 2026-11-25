"""Regression cases in which only the suspended cargo can hit the map."""
from dataclasses import replace
import math

import numpy as np

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import (
    AnalyticSweepSegment,
    CargoFootprint,
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
    VehicleShape,
)
from fwis_backends.multi_4wis.uniform_pipeline import (
    UniformPlanningRequest,
    _build_and_audit,
    expected_audit_request,
    plan_uniform,
    terminal_certificate_key,
)
from fwis_backends.multi_4wis.uniform_curve_refinement import _cargo_collision_witness
from fwis_backends.multi_4wis.uniform_motion import a_straight_edge
from fwis_backends.multi_4wis.uniform_quality import audit_uniform_timed_plan
from fwis_backends.multi_4wis.uniform_timing import timing_motion_limits


def obstacle_grid(x, y):
    cells = np.zeros((120, 120), dtype=bool)
    cells[int(y / 0.05), int(x / 0.05)] = True
    return Grid(cells, 0.05, 0.0, 0.0)


def test_cargo_only_collision_is_rejected_by_final_audit():
    grid = obstacle_grid(3.025, 3.525)
    model = FormationModel([VehicleOffset(1, 1.0, 0.0),
                            VehicleOffset(2, 0.0, 1.0),
                            VehicleOffset(3, 0.0, -1.0)])
    shapes = dict.fromkeys(model.vehicle_ids, VehicleShape(0.738, 0.5))
    request = UniformPlanningRequest(
        grid, model, shapes, FormationPose(1.5, 3.0, 0.0),
        dict.fromkeys(model.vehicle_ids, 0.0), FormationPose(4.5, 3.0, 0.0))
    empty = plan_uniform(request)
    assert empty.executable

    cargo = CargoFootprint(1.1, 1.2, 0.4, 0.0)
    loaded_request = replace(request, cargo_footprint=cargo)
    audit_request = expected_audit_request(loaded_request, empty.plan.edges)
    evaluation = audit_uniform_timed_plan(audit_request, empty.plan, empty.projection, grid)
    assert evaluation.reason == "CARGO_OBB_COLLISION"
    assert evaluation.metrics["cargo_footprint"]["offset_x_m"] == 0.4

    loaded = plan_uniform(loaded_request)
    assert loaded.executable
    assert loaded.plan.edges != empty.plan.edges
    assert loaded.evaluation.metrics["cargo_footprint"]["width_m"] == 1.2


def test_cargo_offset_and_corner_motion_are_checked_during_rotation():
    grid = obstacle_grid(4.375, 3.025)
    model = FormationModel([VehicleOffset(1, 0.0, 0.0)])
    shape = {1: VehicleShape(0.3, 0.3)}
    cargo = CargoFootprint(1.5, 0.4, 0.9, 0.0)
    checker = FormationCollisionChecker(grid, model, shape, 0.0, cargo_footprint=cargo)
    headings = {1: 0.0}
    begin = FormationPose(3.0, 3.0, -math.pi / 4)
    end = FormationPose(3.0, 3.0, math.pi / 4)
    assert checker.static_check(begin, headings).success
    assert checker.static_check(end, headings).success
    raster = checker.fixed_heading_blocked_cells(0.0, headings)
    reference_cell = int(3.0 / grid.res)
    assert raster[reference_cell, reference_cell]

    def stationary_turn(fraction):
        yaw = begin.psi + fraction * (end.psi - begin.psi)
        return FormationState(fraction, FormationPose(3.0, 3.0, yaw), headings)

    segment = AnalyticSweepSegment(0.0, 1.0, stationary_turn,
                                   float, 0.0, math.pi / 2, {1: 0.0})
    result = checker.swept_check_analytic([segment], SweepPolicy())
    assert result.reason == "CARGO_OBB_COLLISION"
    assert 0 < result.station < 1


def test_audit_accepts_equivalent_pi_heading_on_reverse_straight():
    grid = obstacle_grid(.025, .025)
    model = FormationModel([VehicleOffset(1, 0., 0.)])
    start, goal = FormationPose(2., 3., 0.), FormationPose(4., 3., 0.)
    headings = {1: -math.pi}
    request = UniformPlanningRequest(grid, model, {1: VehicleShape(.3, .3)},
                                    start, headings, goal)
    edge = a_straight_edge(model, start, headings, (2., 0.), {1: -1},
                           timing_motion_limits(request.successor_config.timing_limits), True)
    _, _, evaluation = _build_and_audit((edge,), request)
    assert evaluation.feasible


def test_terminal_reuse_binds_exact_request_and_keeps_certified_result():
    grid = obstacle_grid(0.025, 0.025)
    model = FormationModel([VehicleOffset(1, 1.0, 0.0),
                            VehicleOffset(2, 0.0, 1.0),
                            VehicleOffset(3, 0.0, -1.0)])
    shapes = dict.fromkeys(model.vehicle_ids, VehicleShape(0.738, 0.5))
    request = UniformPlanningRequest(
        grid, model, shapes, FormationPose(1.5, 3.0, 0.0),
        dict.fromkeys(model.vehicle_ids, 0.0), FormationPose(4.0, 3.0, 0.0),
        cargo_footprint=CargoFootprint(1.1, 1.2, 0.4, 0.0))
    baseline = plan_uniform(request)
    cached = plan_uniform(replace(request, reuse_terminal_certificate=True))
    assert baseline.executable and cached.executable
    assert cached.stages["terminal_certificate_reused"]
    assert cached.plan.model_sha256() == baseline.plan.model_sha256()
    assert cached.evaluation.as_status() == baseline.evaluation.as_status()
    edges = cached.plan.edges
    key = terminal_certificate_key(request, edges)
    limits = request.successor_config.timing_limits
    for changed in (
        replace(request, grid=obstacle_grid(2.025, 2.025)),
        replace(request, cargo_footprint=replace(request.cargo_footprint, width_m=1.3)),
        replace(request, vehicle_shapes=shapes | {1: VehicleShape(0.740, 0.5)}),
        replace(request, successor_config=replace(request.successor_config,
                                                 timing_limits=replace(limits, v_max=.14))),
        replace(request, goal_qf=FormationPose(4.1, 3.0, 0.0)),
    ):
        assert terminal_certificate_key(changed, edges) != key


def test_cargo_witness_is_a_positive_screen_only():
    model = FormationModel([VehicleOffset(1, 0.0, 0.0)])
    start = FormationPose(1.5, 3.0, 0.0)
    goal = FormationPose(4.5, 3.0, 0.0)
    loaded = UniformPlanningRequest(
        obstacle_grid(3.025, 3.525), model, {1: VehicleShape(0.3, 0.3)},
        start, {1: 0.0}, goal, cargo_footprint=CargoFootprint(1.1, 1.2, 0.4, 0.0))

    class Straight:
        length_m = 3.0
        position_error_bound_m = 1e-9
        angle_error_bound_rad = 1e-9

        def state_at_s(self, station):
            return FormationState(station, FormationPose(start.x + station, start.y, 0.), {1: 0.})

    assert _cargo_collision_witness(loaded, Straight(), .05)
    assert not _cargo_collision_witness(
        replace(loaded, grid=obstacle_grid(.025, .025)), Straight(), .05)
    assert not _cargo_collision_witness(replace(loaded, cargo_footprint=None), Straight(), .05)
