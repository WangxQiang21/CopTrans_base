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
    expected_audit_request,
    plan_uniform,
)
from fwis_backends.multi_4wis.uniform_quality import audit_uniform_timed_plan


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
