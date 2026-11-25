"""Deterministic batch parity against the unchanged Python backends."""

import math

import numpy as np
import pytest

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
from fwis_backends.multi_4wis.uniform_trajectory import _stable_quintic
from fwis_backends.single_continuous_mode_path.core import Action, Pose, advance

native = pytest.importorskip("fwis_planning_core_native")


def test_advance_100000_forward_reverse_and_near_zero_curvature():
    rng = np.random.default_rng(8417)
    count = 100_000
    cases = np.column_stack(
        (
            rng.uniform(-15.0, 15.0, count),
            rng.uniform(-15.0, 15.0, count),
            rng.uniform(-4.0 * math.pi, 4.0 * math.pi, count),
            np.arange(count) % 3,
            rng.uniform(-3.0, 3.0, count),
            rng.uniform(-math.pi, math.pi, count),
        )
    )
    # A-primitives explicitly cross the threshold of the existing straight branch.
    cases[::9, 5] = 0.0
    cases[3::9, 5] = math.nextafter(1e-10, 0.0)
    cases[6::9, 5] = -math.nextafter(1e-10, 1.0)

    calculated = native.advance_batch(cases)
    for row, result in zip(cases.tolist(), calculated.tolist()):
        x, y, yaw, mode, amount, control = row
        expected = advance(Pose(x, y, yaw), Action(int(mode), amount, control))
        assert result == [expected.x, expected.y, expected.th]


def test_progress_and_trajectory_samples_match_python():
    times = np.concatenate((np.linspace(0.0, 3.5, 1000), [math.nextafter(3.5, 0.0)]))
    fractions = times / 3.5
    expected = np.asarray([_stable_quintic(t) for t in fractions.tolist()])
    assert np.allclose(native.quintic_batch(fractions), expected, rtol=0.0, atol=1e-14)

    for mode, amount, angle in ((0, -1.82, 0.15), (1, -0.8, 1.1), (2, 2.7, 0.0)):
        trajectory = native.trajectory_samples((1.3, 4.4, 0.7), (mode, amount, angle), 3.5, times)
        for t, row, (progress, first, *_) in zip(times, trajectory, expected.tolist()):
            pose = advance(Pose(1.3, 4.4, 0.7), Action(mode, amount * progress, angle))
            rate = amount * first / 3.5
            if mode == 0:
                twist = (rate, 0.0, angle * rate)
            elif mode == 1:
                twist = (rate * math.cos(angle), rate * math.sin(angle), 0.0)
            else:
                twist = (0.0, 0.0, rate)
            assert row == pytest.approx((t, pose.x, pose.y, pose.th, *twist), abs=1e-12)


def _kernels(occupied, resolution, origin, offsets, shapes, margin, cargo):
    ids = (3, 11, 19)[: len(offsets)]
    py_grid = Grid(occupied != 0, resolution, *origin)
    py_model = FormationModel([VehicleOffset(vehicle, *offset) for vehicle, offset in zip(ids, offsets)])
    py_shapes = {vehicle: VehicleShape(*shape) for vehicle, shape in zip(ids, shapes)}
    py_cargo = None if cargo is None else CargoFootprint(cargo[2], cargo[3], cargo[0], cargo[1])
    oracle = FormationCollisionChecker(py_grid, py_model, py_shapes, margin, cargo_footprint=py_cargo)
    oracle._native_kernel = None

    footprints = [
        (*offset, length + margin, width + margin)
        for offset, (length, width) in zip(offsets, shapes)
    ]
    envelope = None if cargo is None else (*cargo[:2], cargo[2] + margin, cargo[3] + margin)
    cpp = native.CollisionKernel(occupied, resolution, *origin, footprints, envelope)
    return ids, oracle, cpp


@pytest.mark.parametrize("cargo", [None, (0.36, -0.14, 1.2, 1.8)])
def test_100000_joint_collision_samples_match_reason_and_witness(cargo):
    rng = np.random.default_rng(18823)
    occupied = np.zeros((32, 32), dtype=np.int8)
    occupied[8:15, 16:18] = 100
    occupied[13, 13] = -1
    occupied[6:12, 27] = 100
    ids, oracle, cpp = _kernels(
        occupied,
        0.25,
        (-1.25, -1.75),
        ((0.44, 0.0), (-0.36, 0.53), (-0.36, -0.53)),
        ((0.67, 0.41), (0.63, 0.45), (0.68, 0.42)),
        0.05,
        cargo,
    )
    count = 50_000
    cases = np.column_stack(
        (
            rng.uniform(-1.75, 7.25, count),
            rng.uniform(-2.25, 6.75, count),
            rng.uniform(-3.0 * math.pi, 3.0 * math.pi, count),
            rng.choice([0.0, 0.01, 0.05], count),
            rng.uniform(-2.0 * math.pi, 2.0 * math.pi, (count, len(ids))),
        )
    )
    batch = cpp.check_batch(cases)
    reasons = set()
    for sample, (reason, first, second, _, _) in zip(cases.tolist(), batch.tolist()):
        x, y, yaw, padding, *headings = sample
        result = oracle._check_state(FormationPose(x, y, yaw), dict(zip(ids, headings)), padding)
        actual_reason = native.collision_reason(reason) or None
        assert (reason == 0) == result.success
        assert actual_reason == result.reason
        if reason in (3, 4):
            assert ids[first] == result.vehicle_id
        if reason == 4:
            assert ids[second] == result.other_vehicle_id
        reasons.add(actual_reason)
    assert None in reasons
    assert "MAP_OBB_COLLISION" in reasons
    if cargo:
        assert "CARGO_OBB_COLLISION" in reasons
    assert batch[:, 3].sum() == oracle.statistics()["map_queries"]
    assert batch[:, 4].sum() == oracle.statistics()["obb_pair_queries"]


def test_swept_joint_samples_keep_subdivision_padding_and_failure_witness():
    occupied = np.zeros((32, 32), dtype=np.int8)
    occupied[16, 16] = 100
    ids, oracle, cpp = _kernels(
        occupied, 0.125, (0.0, 0.0), ((0.0, 0.0),), ((0.25, 0.25),),
        0.0, (0.5, 0.0, 0.5, 0.5),
    )

    def state(fraction):
        return FormationState(fraction, FormationPose(1.0 + fraction * 2.0, 2.0, 0.0),
                              {ids[0]: 0.0})

    segment = AnalyticSweepSegment(0.0, 1.0, state, lambda fraction: fraction, 2.0, 0.0, {ids[0]: 0.0})
    policy = SweepPolicy(0.125, 32)
    result = oracle.swept_check_analytic([segment], policy)
    bound, padding, intervals = cpp.sweep_discretization(2.0, 0.0, [0.0], 0.125, 32)
    assert bound == oracle.analytic_segment_bound(segment)
    assert padding == policy.motion_padding_m
    fractions = np.arange(intervals + 1) / intervals
    samples = [(state(fraction).qf.x, 2.0, 0.0, padding, 0.0) for fraction in fractions]
    hits = cpp.check_batch(samples)
    failures = np.nonzero(hits[:, 0])[0]
    assert failures[0] == result.sample_index
    assert native.collision_reason(hits[failures[0], 0]) == result.reason
    assert hits[0, 0] == 0 and hits[-1, 0] == 0
    with pytest.raises(ValueError, match="sample budget"):
        cpp.sweep_discretization(2.0, 0.0, [0.0], 0.125, 1)


def test_invalid_arrays_or_modes_fail_without_returning_partial_outputs():
    with pytest.raises(ValueError, match="Nx6"):
        native.advance_batch(np.zeros((2, 5)))
    with pytest.raises(ValueError, match="mode"):
        native.advance_batch([[1.0, 2.0, 0.0, 65.0, 0.0, 0.0]])
    with pytest.raises(ValueError, match="finite"):
        native.advance_batch([[1.0, 2.0, math.nan, 0.0, 0.0, 0.0]])
