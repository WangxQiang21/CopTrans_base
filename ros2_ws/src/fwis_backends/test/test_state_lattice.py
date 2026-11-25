import math

import numpy as np
import pytest

from fwis_backends.single_continuous_mode_path.core import Action, Grid, Pose, advance, replay, wrap
from fwis_backends.single_continuous_mode_path.retiming import MotionLimits
from fwis_backends.single_continuous_mode_path import state_lattice


LIMITS = MotionLimits(v_max=.16, a_max=.22, jerk_max=.8, omega_max=.40,
                      alpha_max=.45, angular_jerk_max=1., r_min_ack=.90, mode_switch_duration=.70)
START = Pose(2., 2., 0.)
REVERSE_CORRIDOR = ((0, 6, 0, 1.65), (0, 6, 2.35, 6))
LATTICE_CASES = (
    ("straight", START, Pose(3, 2, 0), (), True, True, (0,)),
    ("off_grid_turn", START, advance(START, Action(0, 1.36, .33)), (), True, True, (0,)),
    ("spin_in_place", START, Pose(2, 2, math.pi / 2), (), True, True, (2,)),
    ("side_goal", START, Pose(2, 3.01, 0), (), True, True, (1,)),
    ("reverse_only", Pose(3, 2, 0), Pose(1, 2, 0), REVERSE_CORRIDOR, True, True, (0,)),
    ("reverse_disabled", Pose(3, 2, 0), Pose(1, 2, 0), REVERSE_CORRIDOR, False, False, None),
    ("right_angle", Pose(2, 1, 0), Pose(4, 3, math.pi / 2),
     ((0, 1.45, 0, 6), (1.45, 3, 1.8, 6), (4.55, 6, 0, 6)), True, True, None),
    ("s_corridor", Pose(1, 1, 0), Pose(5, 4, 0),
     ((1.8, 2.2, 0, 2.7), (3.8, 4.2, 3.1, 6)), True, True, None),
    ("disconnecting_wall", Pose(1, 3, 0), Pose(5, 3, 0), ((2.9, 3.1, 0, 6),), True, False, None),
    ("outside_boundary", Pose(1, 3, 0), Pose(6, 3, 0), (), True, False, None),
)


def make_grid(obstacles):
    occupied = np.zeros((120, 120), dtype=bool)
    for x0, x1, y0, y1 in obstacles:
        occupied[round(y0 / .05):round(y1 / .05), round(x0 / .05):round(x1 / .05)] = True
    return Grid(occupied, .05, 0., 0.)


def make_config(name, allow_reverse):
    if name == "s_corridor":
        return state_lattice.LatticeConfig(time_budget_s=1.5, max_expansions=3500,
                                           terminal_radius_m=.9, allow_reverse=allow_reverse)
    return state_lattice.LatticeConfig(time_budget_s=.7, max_expansions=64,
                                       allow_reverse=allow_reverse)


def test_action_library_exactly_closes_grid_and_preserves_radius():
    for step in (.2, .3, .45):
        library = state_lattice.generate_primitives(step, LIMITS.r_min_ack)
        for i, actions in enumerate(library):
            for primitive in actions:
                action = primitive.action
                from_pose = Pose(0, 0, i * math.pi / 2)
                to_pose = advance(from_pose, action)
                assert to_pose.x == pytest.approx(primitive.dx * step)
                assert to_pose.y == pytest.approx(primitive.dy * step)
                assert wrap(to_pose.th - primitive.heading * math.pi / 2) == pytest.approx(0.)
                if action.mode == 0:
                    assert abs(action.control) <= 1 / LIMITS.r_min_ack
                elif action.mode == 1:
                    assert 0 < abs(action.control) <= math.pi / 2 + 1e-10
                else:
                    assert primitive.dx == primitive.dy == action.control == 0


@pytest.mark.parametrize("name,start,goal,obstacles,allow_reverse,success,mode_word", LATTICE_CASES,
                         ids=[case[0] for case in LATTICE_CASES])
def test_certified_incumbent(name, start, goal, obstacles, allow_reverse, success, mode_word):
    callbacks = []
    result = state_lattice.search_lattice(start, goal, make_grid(obstacles), LIMITS,
                                          make_config(name, allow_reverse), callbacks.append)
    assert (result.incumbent is not None) == success
    assert all(candidate.quality["status"] == "FEASIBLE" for candidate in callbacks)
    if not success:
        assert not callbacks and result.first_certified_s is None
        return
    candidate = result.incumbent
    assert result.first_certified_s < make_config(name, allow_reverse).time_budget_s
    assert candidate is callbacks[-1]
    assert all(earlier.timed.duration > later.timed.duration for earlier, later in zip(callbacks, callbacks[1:]))
    terminal = replay(start, candidate.actions)[-1]
    assert math.hypot(terminal.x - goal.x, terminal.y - goal.y) <= 1e-7
    assert abs(wrap(terminal.th - goal.th)) <= 1e-7
    if mode_word is not None:
        assert tuple(a.mode for a in candidate.actions) == mode_word
    if name == "reverse_only":
        assert all(action.amount < 0 for action in candidate.actions)
    if name in ("side_goal", "spin_in_place"):
        first, second = candidate.timed.phases[:2]
        assert first["kind"] == "mode_switch" or (
            first["kind"] == "reconfiguration" and first["mode_switch_event"])
        assert first["duration"] >= LIMITS.mode_switch_duration
        assert second["kind"] in ("translation", "spin")
    if name == "s_corridor":
        assert len({a.mode for a in candidate.actions}) >= 2


def test_failed_refinement_retains_certified_incumbent(monkeypatch):
    refused = []

    def fail(*args, **kwargs):
        refused.append(True)
        raise ValueError("test refinement failure")

    monkeypatch.setattr(state_lattice, "fit_continuous_endpoint", fail)
    config = state_lattice.LatticeConfig(max_expansions=4, refine_continuous=True)
    received = []
    result = state_lattice.search_lattice(START, Pose(2, 3, 0), make_grid(()), LIMITS, config,
                                          received.append)
    assert refused and result.incumbent is received[0]
    assert result.incumbent.quality["status"] == "FEASIBLE"


def test_bad_analytic_timer_reuses_the_legacy_certified_trajectory(monkeypatch):
    from fwis_backends.single_continuous_mode_path.continuous_geometry import plan_from_actions
    from fwis_backends.single_continuous_mode_path.quality import TrajectoryEvaluator

    original = state_lattice.retime_continuous

    def retime(*args, analytic_ackermann=False, **kwargs):
        if analytic_ackermann:
            raise ValueError("experimental timer injected failure")
        return original(*args, **kwargs)

    monkeypatch.setattr(state_lattice, "retime_continuous", retime)
    goal = Pose(3., 2., 0.)
    actions = (Action(0, 1., 0.),)
    plan = plan_from_actions(START, goal, actions)
    evaluator = TrajectoryEvaluator(make_grid(()), START, goal, LIMITS)
    candidate = state_lattice.audited_continuous_candidate(
        plan, actions, evaluator, analytic_ackermann=True)
    assert candidate.quality["status"] == "FEASIBLE"
    assert all(p.get("timing_source") is None for p in candidate.timed.phases)


def test_cancelled_worker_keeps_certified_incumbent_and_stops_after_first():
    received = []
    result = state_lattice.search_lattice(
        START, Pose(3., 2., 0.), make_grid(()), LIMITS, on_certified=received.append,
        cancelled=lambda: bool(received))
    assert result.reason == "CANCELLED"
    assert result.incumbent is received[0]
    assert result.expanded == 1
