"""Application checks for temporal collision certification and native sampling."""

from math import atan2, pi
from types import SimpleNamespace
import pytest

from fwis_bringup.assembly_plan import _schedule, pair_clearance, plan_assembly, sample_track, track_length


def straight(start, end, gear=1):
    yaw = atan2(gear * (end[1] - start[1]), gear * (end[0] - start[0]))
    pieces = [[a + (b - a) * i / 5 for i in range(6)] for a, b in zip(start, end)]
    pieces.append([yaw] * 6)
    phase = dict(mode="A", kind="motion", duration=4., heading0=[yaw] * 3, heading1=[yaw] * 3,
                 gear=[gear] * 3, pieces=[pieces])
    return dict(plan=[phase], delay_s=0., duration_s=phase["duration"], start=(*start, yaw), goal=(*end, yaw),
                request={"limits": {"speed": 1., "yaw_rate": .4, "spin_speed": .4}, "task_duration_cap": 20.})


def test_schedule_detects_between_endpoint_crossing_and_delays():
    first = straight((-1., 0.), (1., 0.))
    second = straight((0., -1.), (0., 1.))
    shape = (.50, .34)
    assert not pair_clearance(first, second, shape)["passed"]
    plan, _ = _schedule({1: first, 2: second}, shape, {})
    assert plan["duration_s"] > first["duration_s"]
    assert all(row["passed"] for row in plan["pair_checks"])


def test_vehicle_that_finished_is_still_an_obstacle():
    first = straight((-1., 0.), (0., 0.))
    second = dict(straight((0., -1.), (0., 0.)), delay_s=5.)
    report = pair_clearance(first, second, (.50, .34))
    assert not report["passed"]
    assert report["time_s"] > second["delay_s"]
    assert sample_track(first, 6.)["pose"] == first["goal"]


def test_shorter_approach_leads_when_both_paths_cross():
    longer = dict(straight((-2., 0.), (2., 0.)), length_m=4.)
    nearer = dict(straight((0., -1.), (0., 1.)), length_m=2.)
    result, _ = _schedule({1: longer, 2: nearer}, (.50, .34), {})
    assert result["delays"][2] == 0.
    assert result["delays"][1] > 0.
    assert all(check["passed"] for check in result["pair_checks"])


def test_reverse_uses_physical_heading_and_signed_body_velocity():
    track = dict(straight((1., 0.), (-1., 0.), gear=-1), delay_s=2.)
    assert sample_track(track, 1.)["kind"] == "hold"
    assert sample_track(track, 4.)["pose"] == pytest.approx((0., 0., 0.))
    assert sample_track(track, 4.)["twist"][0] == pytest.approx(-.9375)
    assert sample_track(track, 6.)["twist"] == (0., 0., 0.)


def test_fixed_assignment_must_be_bijective():
    scene = SimpleNamespace(vehicle_ids=(1, 2, 3), offsets=((1., 0.), (0., 1.), (0., -1.)),
                            assembly_starts=((0., 0., 0.), (2., 0., pi), (2., 2., pi)))
    with pytest.raises(ValueError, match="different dock"):
        plan_assembly(scene, None, (5., 5., 0.), fixed={1: 1, 2: 1, 3: 2})


@pytest.mark.parametrize(("fixed", "slow_vehicle", "expected"), [
    (None, False, (3, 2, 1)),
    (None, True, (2, 1, 3)),
    ({1: 2, 2: 3, 3: 1}, False, (2, 3, 1)),
])
def test_nearest_reachable_mounts_and_fixed_override(monkeypatch, fixed, slow_vehicle, expected):
    """An equal makespan must not turn permutation order into a mount preference."""
    import numpy as np
    from fwis_bringup import assembly_plan as assembly

    starts = ((1.1, 19.9, -pi / 2), (12.1, 22.2, -pi / 2), (20.2, 1.2, pi / 2))
    scene = SimpleNamespace(vehicle_ids=(1, 2, 3), offsets=((1., 0.), (0., 1.), (0., -1.)),
                            assembly_starts=starts, cargo=SimpleNamespace(offset=(.5, 0.)), vehicle_size=(.75, .52))
    grid = (dict(resolution=.05, origin=[0., 0.]), np.zeros((4, 4)))
    monkeypatch.setattr(assembly, "single_model", lambda *_: (.04, dict(speed=1.0)))
    monkeypatch.setattr(assembly, "pair_clearance", lambda *_: dict(passed=True))
    monkeypatch.setattr(assembly, "_guide_track", lambda *_: (None, []))
    solves = set()

    def native(executable, operation, request, budget, **fields):
        if operation == "certify":
            return {"executable": True}
        start, goal = (request[k]["reference"] for k in ("start", "goal"))
        solves.add((start, goal))
        plan = straight(start[:2], goal[:2])["plan"]
        if slow_vehicle and start == starts[1] and goal[1] > 19:
            plan[0]["duration"] = request["task_duration_cap"] + 1
        return dict(plan=plan)

    monkeypatch.setattr(assembly, "native_call", native)
    result = plan_assembly(scene, grid, (10.85, 18.60, 0.), fixed=fixed, executable="mock")
    assert result["status"] == "passed"
    assert tuple(result["assignment"].values()) == expected
    assert result["assignment_policy"] == ("specified" if fixed else "nearest_feasible_path_minimax")
    assert len(solves) == (3 if fixed else 9)
    for i, track in result["tracks"].items():
        assert result["assignment_cost"]["distance_m"][i] == pytest.approx(track_length(track["plan"]))


def test_nearby_mounts_are_compared_by_time_to_finish(monkeypatch):
    import numpy as np
    from fwis_bringup import assembly_plan as assembly

    starts = ((-5., .5, 0.), (-5., -.5, 0.), (6., 0., pi))
    scene = SimpleNamespace(vehicle_ids=(1, 2, 3), offsets=((1., 0.), (0., 1.), (0., -1.)),
                            assembly_starts=starts, cargo=SimpleNamespace(offset=(0., 0.)), vehicle_size=(.75, .52))
    grid = (dict(resolution=.05, origin=[0., 0.]), np.zeros((4, 4)))
    monkeypatch.setattr(assembly, "single_model", lambda *_: (.04, dict(speed=1.0)))
    monkeypatch.setattr(assembly, "pair_clearance", lambda *_: dict(passed=True))

    def native(executable, operation, request, budget, **fields):
        if operation == "certify":
            return {"executable": True}
        start, goal = (request[key]["reference"] for key in ("start", "goal"))
        plan = straight(start[:2], goal[:2])["plan"]
        plan[0]["duration"] = 45. if start == starts[0] and goal[1] > 0. else 10.
        return dict(plan=plan)

    monkeypatch.setattr(assembly, "native_call", native)
    result = plan_assembly(scene, grid, (0., 0., 0.), executable="mock")
    assert result["status"] == "passed"
    assert result["assignment"] == {1: 3, 2: 2, 3: 1}
    assert result["duration_s"] == 10.
