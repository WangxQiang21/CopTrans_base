"""Independent timing, wire identity and unchanged geometry checks."""
from dataclasses import replace
import json
import math
import time

import numpy as np
import pytest

from fwis_backends.multi_4wis.core import Grid
from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
from fwis_backends.multi_4wis.formation_collision import CargoFootprint, FormationCollisionChecker, VehicleShape
from fwis_backends.multi_4wis.formation_continuous_a.geometry import VirtualAPath
from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits, _DYNAMIC_LIMITS
from fwis_backends.multi_4wis.uniform_continuous_corridor import corridor_curves
from fwis_backends.multi_4wis.uniform_curve_refinement import _cruise_profile, _from_start
from fwis_backends.multi_4wis.uniform_mixed_trajectory import (
    MIXED_CLOCK_MODEL_VERSION, UniformMixedTimedPlan,
)
from fwis_backends.multi_4wis.uniform_pipeline import (
    UniformPlanningRequest, _build_and_audit, plan_uniform, refine_certified_plan,
)
from fwis_backends.multi_4wis.uniform_motion import PRIMITIVE_A_STRAIGHT
from fwis_backends.multi_4wis.uniform_progress_clock import reachability_clock
from fwis_backends.multi_4wis.uniform_reachability import (
    REACHABILITY_MODEL_VERSION, UniformReachabilityTimedPlan, certify_clock,
    retime_certified_uniform,
)
from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig, SuccessorConfig
from fwis_backends.multi_4wis.uniform_timing import MotionTiming, SetupTiming


@pytest.mark.parametrize("distance,speed,acceleration,jerk", (
    (.002, .15, .22, 2.), (.10, .1, .22, 2.), (1.5, .15, .22, 2.),
    (1.5, .85, .22, 2.), (20., .85, .22, 2.)))
def test_clock_is_monotone_and_has_exact_stopped_endpoints(distance, speed, acceleration, jerk):
    clock = reachability_clock(distance, speed, acceleration, jerk, min_duration_s=.3)
    assert clock.sample(0., distance)[:3] == (0., 0., 0.)
    assert clock.sample(clock.duration_s, distance)[:3] == (distance, 0., 0.)
    entries = [clock.sample(value * clock.duration_s / 201, distance) for value in range(201)]
    assert all(a[0] <= b[0] for a, b in zip(entries, entries[1:]))
    maxima = tuple(max(abs(row[j]) for row in entries) for j in (1, 2, 3))
    assert all(value <= limit * (1 + 1e-10)
               for value, limit in zip(maxima, (speed, acceleration, jerk)))


@pytest.mark.parametrize("goal", ((5.0, 3.0, 0.0), (4.522379369389306, 3.34843514176563, .45),
                                  (3.0, 6.0, 0.0)))
def test_planner_independently_certifies_versioned_clock_on_original_edges(goal):
    grid = Grid(np.zeros((110, 110), dtype=bool), .1, 0., 0.)
    model = FormationModel([VehicleOffset(1, 1., 0.), VehicleOffset(2, 0., 1.),
                            VehicleOffset(3, 0., -1.)])
    request = UniformPlanningRequest(
        grid, model, dict.fromkeys(model.vehicle_ids, VehicleShape(.738, .500)),
        FormationPose(3., 3., 0.), dict.fromkeys(model.vehicle_ids, 0.), FormationPose(*goal),
        cargo_footprint=CargoFootprint(1.95, 2.70, .5, 0.),
        search_config=GraphSearchConfig(direct_goal_translation=True),
        successor_config=SuccessorConfig(
            timing_limits=FormationTimingLimits(v_max=.16, a_max=.22, r_min_ackermann=.90)))
    baseline = plan_uniform(request)
    result = plan_uniform(replace(request, reachability_retime=True))
    assert baseline.executable and result.executable and result.plan.duration_s < baseline.plan.duration_s * .95
    assert result.projection.model_version == REACHABILITY_MODEL_VERSION
    assert result.evaluation.metrics["dynamic_continuous_certificate"] is True
    assert result.evaluation.metrics["collision_certificate"] is True
    assert result.plan.baseline.model_sha256() == baseline.plan.model_sha256()
    assert retime_certified_uniform(baseline.plan, replace(baseline.evaluation, metrics={})) is None
    decoded = UniformReachabilityTimedPlan.from_model_json(result.plan.to_model_json())
    assert decoded.model_sha256() == result.plan.model_sha256()
    assert decoded.phase_table() == result.plan.phase_table()

    for old, phase in zip(baseline.plan.phases, decoded.phases, strict=True):
        assert old.mode == phase.mode and old.phase_id == phase.phase_id
        if isinstance(phase, SetupTiming):
            assert old.duration_s == phase.duration_s
            assert old.qf == phase.qf and old.demands == phase.demands
        if isinstance(phase, MotionTiming):
            clock = dict(decoded.clocks).get(phase.phase_id)
            if clock is not None:
                bounds = certify_clock(phase, clock, decoded.limits)
                for i in range(120):
                    t = phase.t0_s + phase.duration_s * i / 120
                    state = decoded.state_at(t)
                    replay = result.plan.state_at(t)
                    for vehicle, other in zip(state.vehicles, replay.vehicles, strict=True):
                        vehicle_id = vehicle.vehicle_id
                        assert math.hypot(vehicle.x - other.x, vehicle.y - other.y) < 1e-10
                        values = (vehicle.center_speed_mps, vehicle.center_acceleration_mps2,
                                  vehicle.center_jerk_mps3, abs(vehicle.yaw_rate_radps),
                                  abs(vehicle.yaw_acceleration_radps2), abs(vehicle.yaw_jerk_radps3),
                                  vehicle.wheel_speed_mps, vehicle.lateral_acceleration_mps2)
                        for (name, _, _), actual in zip(_DYNAMIC_LIMITS, values, strict=True):
                            assert actual <= bounds[vehicle_id][name] + 1e-10

    invalid = result.plan.to_model_dict()
    invalid["clocks"][0].update(ramp_s=.001, hold_s=.001, cruise_s=0.)
    with pytest.raises(ValueError):
        UniformReachabilityTimedPlan.from_model_dict(invalid)


def cruise_request(goal):
    model = FormationModel([VehicleOffset(1, 1., 0.), VehicleOffset(2, 0., 1.),
                            VehicleOffset(3, 0., -1.)])
    return UniformPlanningRequest(
        Grid(np.zeros((220, 220), dtype=bool), .1, 0., 0.),
        model, dict.fromkeys(model.vehicle_ids, VehicleShape(.738, .500)),
        FormationPose(8., 6., 0.), dict.fromkeys(model.vehicle_ids, 0.), FormationPose(*goal),
        cargo_footprint=CargoFootprint(1.95, 2.70, .5, 0.),
        search_config=GraphSearchConfig(direct_goal_translation=True),
        successor_config=SuccessorConfig(
            timing_limits=FormationTimingLimits(v_max=.50, a_max=.36, r_min_ackermann=.90)),
        reachability_retime=True)


@pytest.mark.parametrize("goal", ((25., 18., 0.), (12., 18., math.pi)))
def test_c2_corridor_connects_same_and_opposing_parallel_headings(goal):
    base = cruise_request(goal)
    request = replace(base, grid=Grid(np.zeros((300, 350), dtype=bool), .1, 0., 0.),
                      start_qf=FormationPose(8., 6.35, .03),
                      start_headings=dict.fromkeys(base.model.vehicle_ids, .03))
    checker = FormationCollisionChecker(
        request.grid, request.model, request.vehicle_shapes,
        request.successor_config.collision_margin_m, cargo_footprint=request.cargo_footprint)
    profile = next(corridor_curves(request, checker, time.perf_counter() + 4.), None)
    assert profile is not None
    path = VirtualAPath(request.start_qf, profile, request.model)
    endpoint = path.state_at_s(path.length_m).qf
    assert (endpoint.x, endpoint.y, endpoint.psi) == pytest.approx(goal, abs=1e-8)
    assert profile.jet_at(0.).kappa == pytest.approx(0., abs=1e-9)
    assert profile.jet_at(profile.length_m).kappa == pytest.approx(0., abs=1e-9)


def test_close_corridor_goal_uses_two_smooth_approaches_and_a_stopped_reverse_dock():
    request = cruise_request((22.9, 18.2, -.02))
    start = FormationPose(3.9, 5.65, 0.)
    resolution = .05
    occupied = np.zeros((480, 600), dtype=bool)
    x, y = [(np.arange(size) + .5) * resolution for size in (600, 480)]
    for x0, x1, y0, y1 in (
            (7., 18.2, 2., 4.), (7., 16., 7.6, 10.), (11., 16., 12.2, 14.),
            (17., 17.6, 12.8, 13.6), (19.2, 19.4, 9.1, 15.6),
            (23.6, 23.8, 9.1, 15.6), (23.8, 27.8, 20.55, 20.65)):
        occupied[np.ix_((y0 <= y) & (y < y1), (x0 <= x) & (x < x1))] = True
    request = replace(
        request, grid=Grid(occupied, resolution, 0., 0.), start_qf=start,
        search_config=replace(request.search_config, prefer_ackermann=True, convex_corridor=True),
        successor_config=replace(request.successor_config, allow_reverse=True))
    result = plan_uniform(request)
    assert result.executable and result.evaluation.collision_certificate
    assert result.evaluation.metrics["stopped_join_count"] == 1
    plan = result.plan
    assert plan.goal_qf == request.goal_qf
    assert {phase.mode for phase in plan.phases} == {"A"}
    dock, = plan.suffix.edges
    assert dock.primitive == PRIMITIVE_A_STRAIGHT
    assert dock.control.parameter_length == pytest.approx(.5)
    assert dict(dock.directions) == dict.fromkeys(request.model.vehicle_ids, -1)
    profile = plan.curve.path.profile
    curvatures = [profile.jet_at(station).kappa for station in np.linspace(0., profile.length_m, 100)]
    assert min(curvatures) < -.1 and max(curvatures) > .1
    assert all(abs(value) < 1e-8 for value in (curvatures[0], curvatures[-1]))
    seam = plan.curve_phase.end_time
    for side in ("left", "right"):
        assert all(abs(vehicle.longitudinal_speed_mps) < 1e-8
                   for vehicle in plan.state_at(seam, side).vehicles)

    forward_only = replace(request, successor_config=replace(request.successor_config, allow_reverse=False))
    result = plan_uniform(forward_only)
    assert result.executable
    assert all(direction >= 0 for edge in result.plan.edges if edge.mode in ("A", "O")
               for _, direction in edge.directions)


@pytest.mark.parametrize("goal,modes", (
    ((20., 6.26, 0.), ("S", "A")),
    ((8., 18., 0.), ("O",)),
    ((11.15, 6.26, 0.), ("O",)),
))
def test_route_selection_prefers_ackermann_cruise_and_preserves_useful_oblique(goal, modes):
    request = cruise_request(goal)
    baseline = plan_uniform(request)
    assert baseline.executable
    assert tuple(edge.mode for edge in baseline.plan.edges) == ("O",)
    preferred = plan_uniform(replace(
        request, search_config=replace(request.search_config, prefer_ackermann=True)))
    assert preferred.executable
    assert tuple(edge.mode for edge in preferred.plan.edges) == modes
    assert preferred.evaluation.collision_certificate
    assert preferred.plan.goal_qf == request.goal_qf


def test_c2_curve_can_start_and_finish_at_the_requested_poses():
    radius, turn = 5.5, .5
    request = cruise_request((8. + radius * math.sin(turn),
                              6. + radius * (1. - math.cos(turn)), turn))
    baseline = plan_uniform(request)
    assert baseline.executable
    refined, projection, audit = refine_certified_plan(request, baseline.plan, time.perf_counter() + 10.)
    assert audit.collision_certificate and audit.independent_audit
    assert tuple(phase.mode for phase in refined.phases) == ("A",)
    assert not refined.prefix.edges and not refined.suffix.edges
    assert projection.model_version == refined.model_version
    rebuilt = UniformMixedTimedPlan.from_model_json(refined.to_model_json())
    assert rebuilt.model_sha256() == refined.model_sha256()
    assert rebuilt.sample().model_version == projection.model_version
    assert rebuilt.state_at(0.).qf.x == pytest.approx(request.start_qf.x, abs=1e-9)
    endpoint = rebuilt.state_at(rebuilt.duration_s).qf
    assert (endpoint.x, endpoint.y, endpoint.psi) == pytest.approx(
        (request.goal_qf.x, request.goal_qf.y, request.goal_qf.psi), abs=1e-9)
    profile = rebuilt.curve.path.profile
    assert profile.jet_at(0.).kappa == pytest.approx(0., abs=1e-9)
    assert abs(profile.jet_at(rebuilt.curve.path.length_m / 2.).kappa) > .05
    assert profile.jet_at(rebuilt.curve.path.length_m).kappa == pytest.approx(0., abs=1e-9)


def test_long_c2_reachable_clock_round_trips_and_rejects_unsafe_duration():
    request = cruise_request((18., 8., .25))
    baseline = plan_uniform(request)
    assert baseline.executable
    profile = _cruise_profile(request.start_qf, request.goal_qf)
    assert profile is not None
    candidate = _from_start(request, baseline.plan, profile, request.goal_qf, (),
                            _build_and_audit, time.perf_counter() + 10.)
    assert candidate is not None
    plan, projection, audit = candidate
    assert audit.collision_certificate
    assert projection.model_version == MIXED_CLOCK_MODEL_VERSION
    assert plan.duration_s < plan.curve.duration_s
    rebuilt = UniformMixedTimedPlan.from_model_json(plan.to_model_json())
    assert rebuilt.model_sha256() == plan.model_sha256()
    assert rebuilt.default_sample_times() == plan.default_sample_times()
    for fraction in (.0, .12, .55, .75, 1.):
        stamp = fraction * plan.duration_s
        assert rebuilt.state_at(stamp) == plan.state_at(stamp)
    document = plan.to_model_dict()
    document["curve_clock"].update(ramp_s=.001, hold_s=.001, cruise_s=.001)
    with pytest.raises(ValueError, match="C2 clock"):
        UniformMixedTimedPlan.from_model_json(json.dumps(document))
