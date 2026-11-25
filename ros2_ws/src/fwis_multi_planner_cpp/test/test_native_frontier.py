"""Parity between FR04 frontiers, with analytic edges and FR02 certification."""

from dataclasses import replace
import math

import numpy as np
import pytest

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import (
    CargoFootprint,
    FormationCollisionChecker,
    VehicleShape,
)
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_backends.multi_4wis.uniform_search import (
    GraphSearchConfig,
    SuccessorConfig,
    SuccessorGenerator,
    UniformGraphSearch,
    initial_state,
    make_key,
)

native = pytest.importorskip("fwis_multi_frontier_native")


def _request(goal, *, blocked=False, heading=0.0):
    cells = np.zeros((120, 160), dtype=bool)
    if blocked:
        cells[:, 47:52] = True
    grid = Grid(cells, 0.1, 0.0, 0.0)
    model = FormationModel([VehicleOffset(1, 1.0, 0.0), VehicleOffset(2, 0.0, 1.0),
                            VehicleOffset(3, 0.0, -1.0)])
    return UniformPlanningRequest(
        grid, model, dict.fromkeys(model.vehicle_ids, VehicleShape(0.738, 0.5)),
        FormationPose(3.0, 4.0, 0.0), dict.fromkeys(model.vehicle_ids, heading), goal,
        search_config=GraphSearchConfig(max_seconds=30.0, max_expansions=40),
        cargo_footprint=CargoFootprint(1.0, 1.5, 0.32, 0.0),
    )


def _key_tuple(key):
    return (*key.xy_bin, key.psi_bin, key.mode, list(key.wheel_label),
            list(key.heading_label), list(key.a_labels))


def test_native_keys_match_python_wrap_ties_gears_and_capability_labels():
    request = _request(FormationPose(4.0, 4.0, 0.0))
    cfg = SuccessorConfig(xy_bin_m=1.0, psi_bin_rad=0.125, heading_bin_rad=0.125)
    state = initial_state(request.model, request.start_qf, request.start_headings)
    rng = np.random.default_rng(847944)
    for case in range(2000):
        x, y = rng.uniform(-10.0, 10.0, 2)
        if case % 2 == 0:
            x, y = np.trunc(x) + 0.5, np.trunc(y) - 0.5
        exact = replace(
            state,
            qf=FormationPose(x, y, float(rng.uniform(-4.0 * math.pi, 4.0 * math.pi))),
            headings=tuple((vehicle_id, float(rng.uniform(-8.0, 8.0)))
                           for vehicle_id in request.model.vehicle_ids),
            wheel=tuple(replace(wheel, direction=(1 if case % 3 else -1),
                                control=float(rng.uniform(-2.0, 2.0)))
                        for wheel in state.wheel),
            a_labels=("goal_straight", str(case % 4)),
        )
        assert native.make_key(exact, cfg) == _key_tuple(make_key(exact, cfg))


@pytest.mark.parametrize("yaw,goal_yaw,body_offset,allow_reverse", [
    (0.0, 0.0, 0.0, True),
    (0.27, 0.90, math.pi, True),
    (0.13, 0.50, math.pi, False),
    (-0.2, math.pi - 0.2, 0.02, True),
])
def test_complete_candidate_order_costs_and_rejections_match(yaw, goal_yaw, body_offset, allow_reverse):
    request = _request(FormationPose(5.0, 5.0, goal_yaw))
    occupied = np.array(request.grid.occ, copy=True)
    occupied[54:57, 41:45] = True
    grid = Grid(occupied, request.grid.res, request.grid.ox, request.grid.oy)
    opts = replace(request.successor_config, allow_reverse=allow_reverse,
                   arc_pivots=((0.0, 2.0), (-0.0, 1.0), (-2.0, 0.0)))
    start = initial_state(request.model, FormationPose(3.0, 4.0, yaw),
                          dict.fromkeys(request.model.vehicle_ids, yaw + body_offset),
                          config=opts, goal_qf=request.goal_qf)

    def generate(use_cpp):
        checker = FormationCollisionChecker(
            grid, request.model, request.vehicle_shapes, opts.collision_margin_m,
            cargo_footprint=request.cargo_footprint)
        generator = SuccessorGenerator(request.model, checker, opts, native_candidates=use_cpp)
        return generator.generate(start, request.goal_qf), checker.statistics()

    expected, baseline_counts = generate(False)
    actual, native_counts = generate(True)
    assert expected.candidate_count > 0
    assert actual.candidate_count == expected.candidate_count
    assert actual.rejections == expected.rejections
    assert native_counts == baseline_counts
    assert actual.successors == expected.successors


@pytest.mark.parametrize("mode", [None, "A", "O"])
def test_direct_translation_uses_original_order(mode):
    request = _request(FormationPose(5.0, 4.0, 0.0))
    checker = FormationCollisionChecker(request.grid, request.model, request.vehicle_shapes, .05,
                                       cargo_footprint=request.cargo_footprint)
    start = initial_state(request.model, request.start_qf, request.start_headings,
                          config=request.successor_config, goal_qf=request.goal_qf)
    reference = SuccessorGenerator(request.model, checker, request.successor_config)
    cpp = SuccessorGenerator(request.model, checker, request.successor_config, native_candidates=True)
    options = {"direct_goal_translation": True, "translation_mode": mode}
    first = reference.generate(start, request.goal_qf, **options)
    second = cpp.generate(start, request.goal_qf, **options)
    assert first.candidate_count == second.candidate_count and first.candidate_count > 0
    assert first.rejections == second.rejections
    assert first.successors == second.successors


@pytest.mark.parametrize(
    ("goal", "blocked", "direct", "expected_modes"),
    [
        (FormationPose(5.0, 4.0, 0.0), False, False, {"A"}),
        (FormationPose(3.0, 5.0, 0.0), False, False, {"O"}),
        (FormationPose(3.0 + 2.0 * math.sin(math.pi / 18.0),
                       4.0 + 2.0 * (1.0 - math.cos(math.pi / 18.0)),
                       math.pi / 18.0), False, True, {"S", "A"}),
        (FormationPose(11.0, 4.0, 0.0), False, True, {"A"}),
        (FormationPose(8.0, 4.0, 0.0), True, False, None),
    ],
)
def test_graph_results_match_model_hash_modes_certification_and_failure(
    goal, blocked, direct, expected_modes
):
    request = _request(goal, blocked=blocked)
    cfg = replace(request.search_config, direct_goal_translation=direct)
    reference = plan_uniform(replace(request, search_config=cfg))
    actual = plan_uniform(replace(request, search_config=replace(cfg, native_frontier=True)))
    assert reference.executable == actual.executable == (expected_modes is not None)
    assert actual.reason == reference.reason
    assert actual.search.frontier_backend == "cpp"
    for attr in ("expansions", "accepted_successors", "stale_heap_entries",
                 "duplicate_key_rejections", "direct_goal_translation"):
        assert getattr(actual.search, attr) == getattr(reference.search, attr)
    if expected_modes is not None:
        assert actual.plan.model_sha256() == reference.plan.model_sha256()
        assert {edge.mode for edge in actual.plan.edges} == expected_modes
        assert actual.evaluation.as_status() == reference.evaluation.as_status()


@pytest.mark.parametrize("native_frontier", [False, True])
def test_audit_cancellation_and_expansion_budget_still_gate_frontier(native_frontier):
    request = _request(FormationPose(4.0, 4.0, 0.0))
    cfg = replace(request.search_config, native_frontier=native_frontier)
    checker = FormationCollisionChecker(
        request.grid, request.model, request.vehicle_shapes, 0.05,
        cargo_footprint=request.cargo_footprint)
    start = initial_state(request.model, request.start_qf, request.start_headings,
                          config=request.successor_config, goal_qf=request.goal_qf)

    def run(search_config, **kwargs):
        return UniformGraphSearch(request.model, checker, request.successor_config,
                                  search_config).search(start, request.goal_qf, **kwargs)

    assert run(cfg, cancelled=lambda: True).reason == "REQUEST_CANCELLED"
    assert run(replace(cfg, max_expansions=0)).reason == "EXPANSION_BUDGET_EXHAUSTED"
    assert run(cfg).reason == "TERMINAL_VALIDATOR_REQUIRED"

    offered = []

    def reject(edges):
        offered.append(edges)
        return False, {"failure_code": "FIXTURE_REJECTED"}

    rejected = run(cfg, terminal_validator=reject)
    assert rejected.status != "SUCCESS"
    assert offered and rejected.terminal_rejection_histogram["FIXTURE_REJECTED"] > 0


def test_native_terminal_bridge_exposes_deadline_even_when_audit_accepts():
    times = iter((0.75, 1.0))
    observed_edges = []
    edges = ("fixture",)

    def validate(value):
        observed_edges.append(value)
        return True, {"collision_certificate": True}

    verdict = native.SearchBudget(4, 1.0).audit_terminal(0.0, lambda: next(times), validate, edges)
    assert observed_edges == [edges]
    assert verdict == (True, {"collision_certificate": True}, 0.75, 1.0, True)
