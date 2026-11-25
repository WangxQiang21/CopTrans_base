"""Offline Phase7 admission and failure paths with a real certified planner."""

from dataclasses import replace
from threading import Event
from time import perf_counter, sleep

import numpy as np
import pytest

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
from fwis_backends.multi_4wis.local_replan_protocol import (
    Acknowledgement,
    CertifiedPlan,
    LocalReplanCoordinator,
    MapRevision,
    stopped_window,
)
from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig, SuccessorConfig


def reference_request():
    model = FormationModel([VehicleOffset(1, 1.0, 0.0), VehicleOffset(2, 0.0, 1.0),
                            VehicleOffset(3, 0.0, -1.0)])
    limits = FormationTimingLimits(v_max=.16, a_max=.22, yaw_rate_max=.4, yaw_accel_max=.45,
                                   r_min_ackermann=.90, mode_switch_duration=.70)
    return UniformPlanningRequest(
        Grid(np.zeros((90, 90), dtype=bool), .1, 0.0, 0.0),
        model, dict.fromkeys(model.vehicle_ids, VehicleShape(.738, .50)),
        FormationPose(3.0, 4.0, 0.0), dict.fromkeys(model.vehicle_ids, 0.0),
        FormationPose(4.522379369389306, 4.34843514176563, .45),
        plan_id=17,
        search_config=GraphSearchConfig(direct_goal_translation=True),
        successor_config=SuccessorConfig(timing_limits=limits),
        cargo_footprint=CargoFootprint(1.95, 2.70, .50, 0.0),
    )


def local_window(snapshot, now_ns):
    return stopped_window(
        snapshot, now_ns, min_lead_ns=200_000_000, switch_dwell_ns=600_000_000,
        max_path_m=2.0, max_motion_s=35.0, max_lookahead_ns=20_000_000_000)


@pytest.fixture(scope="module")
def global_plan():
    request = reference_request()
    result = plan_uniform(request)
    active = CertifiedPlan.capture(result, MapRevision.observe(8, request), 1_000_000_000)
    window = local_window(active, 1_000_000_000)
    assert window is not None
    return request, active, window


class Clock:
    value = 1_000_000_000

    def __call__(self):
        return self.value


def prepared(coordinator):
    deadline = perf_counter() + 5.0
    while perf_counter() < deadline:
        result = coordinator.poll()
        if result.action == "PREPARE":
            return result.preparation
        if result.reason != "COMPUTING_LOCAL_WINDOW":
            raise AssertionError(result)
        sleep(.005)
    raise AssertionError("local planner never prepared")


def test_real_local_certificate_prepare_ack_and_shared_epoch(global_plan):
    request, old, window = global_plan
    clock = Clock()
    coordinator = LocalReplanCoordinator(old, clock=clock)
    try:
        coordinator.begin(request, window, plan_id=18, computation_budget_s=1.5, max_expansions=40)
        proposal = prepared(coordinator)
        assert proposal.expected_ids == (1, 2, 3)
        assert proposal.epoch_ns == window.epoch_ns
        assert proposal.plan_id > old.plan.plan_id and proposal.model_sha256 != old.model_sha256
        assert not coordinator.acknowledge(Acknowledgement(1, proposal, clock(), True))
        clock.value = window.stop_ns + 100_000_000
        for wrong in (replace(proposal, plan_id=17), replace(proposal, model_sha256=old.model_sha256),
                      replace(proposal, epoch_ns=proposal.epoch_ns - 1),
                      replace(proposal, revision=replace(proposal.revision, number=3))):
            assert not coordinator.acknowledge(Acknowledgement(1, wrong, clock(), True))
        assert not coordinator.acknowledge(Acknowledgement(4, proposal, clock(), True))
        assert not coordinator.acknowledge(Acknowledgement(1, proposal, clock(), False))
        for vehicle in (1, 2):
            assert coordinator.acknowledge(Acknowledgement(vehicle, proposal, clock(), True))
        assert coordinator.poll().action == "PREPARE"
        assert coordinator.acknowledge(Acknowledgement(3, proposal, clock(), True))
        switched = coordinator.poll()
        assert switched.action == "SWITCH" and switched.preparation == proposal
        assert coordinator.poll().action == "HOLD_OLD"
        clock.value = window.epoch_ns
        assert coordinator.poll().action == "EXECUTE_ACTIVE"
        assert coordinator.active.model_sha256 == proposal.model_sha256
        clock.value = coordinator.active.end_ns
        assert coordinator.poll().action == "GROUP_STOP"
    finally:
        coordinator.close()


def test_stale_acks_and_missing_vehicle_cannot_open_group_gate(global_plan):
    request, old, window = global_plan
    clock = Clock()
    coordinator = LocalReplanCoordinator(old, clock=clock)
    try:
        coordinator.begin(request, window, plan_id=18, computation_budget_s=1.5, max_expansions=40)
        proposal = prepared(coordinator)
        clock.value = window.stop_ns
        for vehicle in (1, 2):
            assert coordinator.acknowledge(Acknowledgement(vehicle, proposal, clock(), True))
        clock.value += 250_000_000
        assert not coordinator.acknowledge(Acknowledgement(3, proposal, window.stop_ns, True))
        assert coordinator.acknowledge(Acknowledgement(3, proposal, clock(), True))
        assert coordinator.poll().action == "PREPARE"
        clock.value = window.epoch_ns - 100_000_000
        state = coordinator.poll()
        assert state.action == "HOLD_OLD" and state.reason == "ACK_DEADLINE"
    finally:
        coordinator.close()


def test_map_revision_invalidates_even_a_complete_prepare(global_plan):
    request, old, window = global_plan
    clock = Clock()
    coordinator = LocalReplanCoordinator(old, clock=clock)
    try:
        coordinator.begin(request, window, plan_id=18, computation_budget_s=1.5, max_expansions=40)
        proposal = prepared(coordinator)
        clock.value = window.stop_ns
        for vehicle in (1, 2, 3):
            assert coordinator.acknowledge(Acknowledgement(vehicle, proposal, clock(), True))
        assert coordinator.poll().action == "SWITCH"
        coordinator.observe_map(replace(old.revision, number=old.revision.number + 1))
        assert coordinator.poll().action == "GROUP_STOP"
        assert not coordinator.acknowledge(Acknowledgement(1, proposal, clock(), True))
    finally:
        coordinator.close()


def test_pending_worker_never_blocks_poll_and_timeout_holds_valid_old(global_plan):
    request, old, window = global_plan
    clock = Clock()
    release = Event()
    entered = Event()

    def stalled(request):
        entered.set()
        release.wait(3.0)
        return plan_uniform(request)

    coordinator = LocalReplanCoordinator(old, clock=clock, planner=stalled)
    try:
        coordinator.begin(request, window, plan_id=18, computation_budget_s=.2, max_expansions=40)
        assert entered.wait(2.0)
        assert coordinator.poll().action == "HOLD_OLD"
        assert not release.is_set()
        clock.value += 210_000_000
        state = coordinator.poll()
        assert state.action == "HOLD_OLD" and state.reason == "PLANNER_TIMEOUT"
        with pytest.raises(ValueError, match="still exiting"):
            coordinator.begin(request, window, plan_id=19, computation_budget_s=.2, max_expansions=40)
        clock.value -= 100_000_000
        assert coordinator.poll().reason == "CLOCK_JUMP"
    finally:
        release.set()
        coordinator.close()


@pytest.mark.parametrize("certificate", ["dynamic_continuous_certificate", "map_version_checked"])
def test_capture_requires_continuous_dynamics_and_current_map_audit(global_plan, certificate):
    request, old, _ = global_plan
    outcome = plan_uniform(request)
    metrics = {**outcome.evaluation.metrics, certificate: False}
    incomplete = replace(outcome, evaluation=replace(outcome.evaluation, metrics=metrics))
    with pytest.raises(ValueError, match="expected a certified"):
        CertifiedPlan.capture(incomplete, old.revision, old.epoch_ns)


def test_window_and_map_are_bound_to_the_authoritative_request(global_plan):
    request, old, window = global_plan
    clock = Clock()
    coordinator = LocalReplanCoordinator(old, clock=clock)
    try:
        with pytest.raises(ValueError, match="differs"):
            coordinator.begin(replace(request, cargo_footprint=replace(request.cargo_footprint, width_m=2.5)),
                              window, plan_id=18, computation_budget_s=1.5, max_expansions=40)
        with pytest.raises(ValueError, match="stationary"):
            translated = replace(window, stop_ns=window.stop_ns - 1)
            coordinator.begin(request, translated, plan_id=18, computation_budget_s=1.5, max_expansions=40)
    finally:
        coordinator.close()


@pytest.mark.parametrize("bad_candidate,reason", [("none", "EXPANSION_BUDGET_EXHAUSTED"),
                                                 ("cargo", "LOCAL_WINDOW_REQUEST_MISMATCH")])
def test_no_plan_or_different_cargo_certificate_leaves_old_plan_active(global_plan, bad_candidate, reason):
    request, old, window = global_plan
    clock = Clock()

    def probe(value):
        if bad_candidate == "none":
            return plan_uniform(replace(value, search_config=replace(value.search_config, max_expansions=0)))
        return plan_uniform(replace(value, cargo_footprint=replace(value.cargo_footprint, width_m=2.5)))

    coordinator = LocalReplanCoordinator(old, clock=clock, planner=probe)
    try:
        coordinator.begin(request, window, plan_id=18, computation_budget_s=1.5, max_expansions=40)
        deadline = perf_counter() + 5.0
        while perf_counter() < deadline:
            state = coordinator.poll()
            assert state.action == "HOLD_OLD"
            if state.reason == reason:
                break
            sleep(.005)
        assert state.reason == reason
        assert coordinator.active.model_sha256 == old.model_sha256
    finally:
        coordinator.close()
