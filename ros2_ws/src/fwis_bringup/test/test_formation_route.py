from hashlib import sha256

from builtin_interfaces.msg import Duration, Time
from fwis_v3_msgs.msg import (FormationPlanRequest, FormationPlaybackPlan, FormationTimedPoint,
                             FormationTimedTrajectory, FormationVehicleSpec, FormationVehicleTrajectory)
from geometry_msgs.msg import Point, Pose2D
import pytest
from std_msgs.msg import Header

from fwis_backends.multi_4wis.formation_continuous_a import contracts
from fwis_bringup.formation_route import make_routes


def fixture():
    header = Header(frame_id="map", stamp=Time(sec=5))
    offsets = ((1., 0.), (0., 1.), (0., -1.))
    start, end = Pose2D(x=4., y=5.), Pose2D(x=4.1, y=5.)
    specs = [
        FormationVehicleSpec(vehicle_id=i, offset_in_formation=Point(x=dx, y=dy),
                             start_center=Pose2D(x=start.x + dx, y=start.y + dy),
                             goal_center=Pose2D(x=end.x + dx, y=end.y + dy), goal_heading_free=True,
                             body_length_m=.738, body_width_m=.50)
        for i, (dx, dy) in enumerate(offsets, 1)
    ]
    request = FormationPlanRequest(header=header, plan_id=7, formation_id="cargo_wedge",
                                   formation_start=start, formation_goal=end, vehicles=specs)
    times = [Duration(), Duration(nanosec=20000000)]
    tracks = [FormationVehicleTrajectory(vehicle_id=v.vehicle_id, points=[
        FormationTimedPoint(time_from_start=times[0], center_pose=v.start_center),
        FormationTimedPoint(time_from_start=times[1], center_pose=v.goal_center)])
        for v in reversed(specs)]
    timed = FormationTimedTrajectory(header=header, plan_id=7, formation_id=request.formation_id,
                                     vehicles=specs, formation_poses=[start, end],
                                     times_from_start=times, trajectories=tracks)
    model = "{\"synthetic_fixture\":true}"
    envelope = FormationPlaybackPlan(
        header=header, plan_id=7, formation_id=request.formation_id, trajectory=timed,
        backend=contracts.BACKEND, semantic_version=contracts.PLAYBACK_SEMANTIC_VERSION,
        model_version=contracts.TIMED_MODEL_VERSION, certification=contracts.PLAYBACK_CERTIFICATION,
        model_json=model, model_sha256=sha256(model.encode()).hexdigest())
    return request, envelope


def test_routes_follow_ids_and_preserve_the_common_grid():
    request, envelope = fixture()
    start = Time(sec=7)
    routes = make_routes(envelope, request, start)
    assert set(routes) == {1, 2, 3}
    for vehicle in request.vehicles:
        track = routes[vehicle.vehicle_id]
        assert track.header.stamp == start and track.plan_id == request.plan_id
        assert track.points[0].center_pose == vehicle.start_center
        assert [point.time_from_start for point in track.points] == envelope.trajectory.times_from_start
    assert envelope.header.stamp.sec == 5


@pytest.mark.parametrize("corruption,match", [
    ("time", "differs from common grid"),
    ("vehicle", "duplicate trajectory vehicle_id"),
    ("plan", "identity"),
    ("mode", "mode change"),
])
def test_router_rejects_the_whole_envelope_on_inconsistent_data(corruption, match):
    request, envelope = fixture()
    track = envelope.trajectory.trajectories[0]
    if corruption == "time":
        track.points[-1].time_from_start = Duration(nanosec=21000000)
    elif corruption == "vehicle":
        track.vehicle_id = envelope.trajectory.trajectories[1].vehicle_id
    elif corruption == "plan":
        envelope.trajectory.plan_id += 1
    elif corruption == "mode":
        track.points[-1].mode = FormationTimedPoint.SPIN
    with pytest.raises(ValueError, match=match):
        make_routes(envelope, request, Time(sec=7))


def test_native_requests_require_two_current_clocks_and_the_same_map():
    import json
    from fwis_bringup.native_readiness import NativeReadiness

    gate = NativeReadiness()
    stamp = 2_000_000_000

    def report(role, **changes):
        data = {"ready": True, "clock_ns": stamp, "map_sha256": "a" * 64}
        gate.observe(role, json.dumps(data | changes), wall=10.)

    report("planner")
    assert not gate.ready(stamp, 10.)
    report("executor", clock_ns=0)
    assert not gate.ready(stamp, 10.)
    report("executor", map_sha256="b" * 64)
    assert not gate.ready(stamp, 10.)
    report("executor")
    assert gate.ready(stamp, 10.)
    assert not gate.ready(stamp, 10.21)
    assert not gate.ready(stamp - 201_000_000, 10.)
    assert not gate.ready(stamp + 201_000_000, 10.)
    report("planner", ready=False)
    assert not gate.ready(stamp, 10.)
    report("planner")
    gate.observe("executor", "corrupt", wall=10.)
    assert not gate.ready(stamp, 10.)
