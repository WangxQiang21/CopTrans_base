"""Differential tests for the native joint-state formation collision checker."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from fwis_backends.multi_4wis import formation_collision
from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import (
    AnalyticSweepSegment,
    CargoFootprint,
    FormationCollisionChecker,
    FormationState,
    SweepPolicy,
    VehicleShape,
    grid_from_occupancy_grid,
)
from fwis_backends.multi_4wis.uniform_motion import (
    a_arc_edge,
    a_arc_required_headings,
    a_straight_edge,
    o_translation_edge,
)
from fwis_backends.multi_4wis.uniform_search import analytic_sweep_segment


pytestmark = pytest.mark.skipif(
    getattr(formation_collision, "_formation_collision_native", None) is None,
    reason="formation collision extension was not built",
)


def _checker_pair(grid, model, shapes, margin=0.0, cargo=None):
    native = FormationCollisionChecker(grid, model, shapes, margin, cargo_footprint=cargo)
    reference = FormationCollisionChecker(grid, model, shapes, margin, cargo_footprint=cargo)
    assert native._native_kernel is not None
    reference._native_kernel = None
    return native, reference


def _static(native, reference, pose, headings, reason):
    actual = native.static_check(pose, headings)
    assert actual == reference.static_check(pose, headings)
    assert actual.reason == reason
    assert native.statistics() == reference.statistics()


def test_map_edges_touching_and_unknown_occupied_cell():
    size = 32
    data = [0] * (size * size)
    data[4 * size + 8] = -1
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    info = SimpleNamespace(width=size, height=size, resolution=0.125, origin=origin)
    message = SimpleNamespace(header=SimpleNamespace(frame_id="map"), info=info, data=data)
    grid, error = grid_from_occupancy_grid(message)
    assert error is None
    model = FormationModel([VehicleOffset(0, 0.0, 0.0)])
    native, reference = _checker_pair(grid, model, {0: VehicleShape(0.5, 0.25)})
    headings = {0: 0.0}

    _static(native, reference, FormationPose(0.25, 0.5625, 0.0), headings, "MAP_OBB_COLLISION")
    _static(native, reference, FormationPose(math.nextafter(0.25, math.inf), 0.5625, 0.0),
            headings, None)
    _static(native, reference, FormationPose(0.75, 0.5625, 0.0), headings, "MAP_OBB_COLLISION")
    _static(native, reference, FormationPose(math.nextafter(0.75, -math.inf), 0.5625, 0.0),
            headings, None)
    _static(native, reference, FormationPose(1.0625, 0.5625, 0.0), headings, "MAP_OBB_COLLISION")


def test_grid_precision_falls_back_when_cell_centers_collapse():
    # At this coordinate scale several neighboring cell centers round to the
    # same float. Exact geometry stays with the existing Python checker.
    cells = np.zeros((100, 160), dtype=bool)
    cells[50, 45] = True
    grid = Grid(cells, 0.01, 1e15, 0.0)
    model = FormationModel([VehicleOffset(1, 0.0, 0.0)])
    checker = FormationCollisionChecker(grid, model, {1: VehicleShape(0.001, 0.001)}, 0.0)
    assert checker._native_kernel is None
    assert checker.static_check(
        FormationPose(1e15 + 0.5, 0.505, 0.0), {1: 0.0}).reason == "MAP_OBB_COLLISION"


def test_touching_vehicles_and_cargo_first_failure_order():
    cells = np.zeros((32, 32), dtype=bool)
    vehicle_ids = (0, 2**32 - 1)
    model = FormationModel([VehicleOffset(vehicle_ids[1], 0.5, 0.0),
                            VehicleOffset(vehicle_ids[0], 0.0, 0.0)])
    shapes = dict.fromkeys(vehicle_ids, VehicleShape(0.5, 0.25))
    headings = dict.fromkeys(reversed(vehicle_ids), 0.0)
    pose = FormationPose(2.0, 2.0, 0.0)
    native, reference = _checker_pair(Grid(cells, 0.125, 0.0, 0.0), model, shapes)
    _static(native, reference, pose, headings, "VEHICLE_OBB_COLLISION")
    pair = native.static_check(pose, headings)
    assert (pair.vehicle_id, pair.other_vehicle_id) == vehicle_ids

    cells[16, 16] = True
    occupied_grid = Grid(cells, 0.125, 0.0, 0.0)
    native, reference = _checker_pair(occupied_grid, model, shapes)
    _static(native, reference, pose, headings, "MAP_OBB_COLLISION")
    assert native.statistics()["obb_pair_queries"] == 0

    native, reference = _checker_pair(occupied_grid, model, shapes, cargo=CargoFootprint(0.75, 0.5))
    _static(native, reference, pose, headings, "CARGO_OBB_COLLISION")
    assert native.statistics()["map_queries"] == 1
    assert native.statistics()["obb_pair_queries"] == 0


@pytest.mark.parametrize("budget,reason", [(100, "CARGO_OBB_COLLISION"),
                                          (1, "SWEEP_SAMPLE_BUDGET_EXCEEDED")])
def test_analytic_sweep_preserves_first_witness_trace_and_budget(budget, reason):
    cells = np.zeros((32, 32), dtype=bool)
    cells[16, 16] = True
    model = FormationModel([VehicleOffset(7, 0.0, 0.0)])
    native, reference = _checker_pair(
        Grid(cells, 0.125, 0.0, 0.0), model, {7: VehicleShape(0.25, 0.25)},
        cargo=CargoFootprint(0.5, 0.5, 0.5, 0.0),
    )

    def segment(start, end, station):
        span = end - start
        return AnalyticSweepSegment(
            station, station + 1.0,
            lambda f: FormationState(station + f, FormationPose(start + f * span, 2.0, 0.0), {7: 0.0}),
            lambda f: (station + f) * 2.0, span, 0.0, {7: 0.0},
        )

    segments = [segment(0.75, 0.875, 3.0), segment(0.875, 3.0, 4.0)]
    policy = SweepPolicy(max_corner_step_m=0.125, max_samples_per_segment=budget)
    actual_trace, expected_trace = [], []
    actual = native.swept_check_analytic(segments, policy, trace=actual_trace)
    expected = reference.swept_check_analytic(segments, policy, trace=expected_trace)
    assert actual == expected and actual_trace == expected_trace
    assert actual.reason == reason and actual.segment_index == 1
    assert 4.0 <= actual.station < 5.0
    assert actual.time_s == 2.0 * actual.station
    assert native.statistics() == reference.statistics()


@pytest.mark.parametrize("blocked,budget", [(False, 100), (True, 100), (True, 1)])
def test_batched_stationary_spin_keeps_witness_and_query_counts(blocked, budget):
    cells = np.zeros((32, 32), dtype=bool)
    cells[16, 20] = blocked
    model = FormationModel([VehicleOffset(7, 0.0, 0.0)])
    native, reference = _checker_pair(
        Grid(cells, 0.125, 0.0, 0.0), model, {7: VehicleShape(1.2, 0.15)},
    )
    assert hasattr(native._native_kernel, "check_spin_samples"), "rebuild the collision extension"
    qf = FormationPose(2.0, 2.0, 0.0)
    before, after = {7: -math.pi / 4}, {7: math.pi / 4}
    length = math.pi / 2

    def state(fraction):
        progress = fraction * length
        return FormationState(fraction, qf, {7: before[7] + (after[7] - before[7]) * progress / length})

    segment = AnalyticSweepSegment(
        0.0, 1.0, state, lambda fraction: 2.0 * fraction, 0.0, 0.0, {7: length},
    )
    policy = SweepPolicy(0.01, budget)
    expected = reference.swept_check_analytic([segment], policy)
    actual = native.swept_check_analytic([segment], policy, _stationary_spin=(length, before, after))
    assert actual == expected
    assert actual.reason == ("SWEEP_SAMPLE_BUDGET_EXCEEDED" if budget == 1 else
                             "MAP_OBB_COLLISION" if blocked else None)
    assert native.statistics() == reference.statistics()


@pytest.mark.parametrize("kind", ["straight", "arc", "oblique"])
def test_batched_analytic_edge_keeps_cargo_mid_edge_witness(kind):
    model = FormationModel([VehicleOffset(17, 0.0, 0.0)])
    start = FormationPose(2.0, 2.0, 0.0)
    gear = {17: 1}
    if kind == "arc":
        headings = a_arc_required_headings(model, start, .8, (0.0, 2.0), gear)
        edge = a_arc_edge(model, start, headings, .8, (0.0, 2.0), gear)
    elif kind == "straight":
        edge = a_straight_edge(model, start, {17: 0.0}, (2.0, 0.0), gear)
    else:
        edge = o_translation_edge(model, start, {17: 0.0}, (0.0, 2.0), gear, {17: math.pi / 2})
    midpoint = edge.state_at(edge.control.parameter_length / 2).qf
    res = .0625
    occupancy = np.zeros((100, 100), dtype=bool)
    occupancy[int(midpoint.y / res), int(midpoint.x / res)] = True
    native, python = _checker_pair(
        Grid(occupancy, res, 0.0, 0.0), model, {17: VehicleShape(.1, .1)},
        cargo=CargoFootprint(.15, .16),
    )
    for pose, headings in ((edge.start_qf, edge.start_headings), (edge.end_qf, edge.end_headings)):
        assert python.static_check(pose, dict(headings)).success
        assert native.static_check(pose, dict(headings)).success
    segment = analytic_sweep_segment(edge, 3.0)
    policy = SweepPolicy(.01, 1000)
    expected = python.swept_check_analytic([segment], policy)
    got = native.swept_check_analytic([segment], policy, _exact_edge=edge)
    assert got == expected
    assert got.reason == "CARGO_OBB_COLLISION" and 0.0 < got.station < 1.0
    assert native.statistics() == python.statistics()


@pytest.mark.parametrize("cargo", [None, CargoFootprint(0.8, 0.95, 0.43, -0.28)])
def test_seeded_rotated_geometry_on_strided_map(cargo):
    rng = np.random.default_rng(8464)
    backing = rng.random((48, 130)) < 0.008
    cells = backing[:, ::2]
    cells[15:20, 24:29] = True
    cells[23, [0, 63, 64]] = True
    grid = Grid(cells, 0.125, -1.5, 2.25)
    assert not grid.occ.flags.c_contiguous
    model = FormationModel([VehicleOffset(9, 0.6, 0.0),
                            VehicleOffset(3, -0.55, 0.45),
                            VehicleOffset(18, -0.55, -0.45)])
    shapes = {3: VehicleShape(0.5, 0.25), 9: VehicleShape(0.65, 0.4), 18: VehicleShape(0.55, 0.35)}
    native, reference = _checker_pair(grid, model, shapes, margin=0.05, cargo=cargo)
    results = set()
    for _ in range(160):
        pose = FormationPose(rng.uniform(-2.0, 7.0), rng.uniform(1.75, 8.75), rng.uniform(-math.pi, math.pi))
        headings = {vehicle_id: rng.uniform(-math.pi, math.pi) for vehicle_id in model.vehicle_ids}
        report = native.static_check(pose, headings)
        assert report == reference.static_check(pose, headings)
        results.add(report.reason)
    assert {None, "MAP_OBB_COLLISION"} <= results
    if cargo is not None:
        assert "CARGO_OBB_COLLISION" in results
    assert native.statistics() == reference.statistics()
