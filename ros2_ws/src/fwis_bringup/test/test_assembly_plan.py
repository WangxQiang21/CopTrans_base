"""Application checks for temporal collision certification and native sampling."""

from math import atan2, pi
from types import SimpleNamespace
import pytest

from fwis_bringup.assembly_plan import _schedule, pair_clearance, plan_assembly, sample_track


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
