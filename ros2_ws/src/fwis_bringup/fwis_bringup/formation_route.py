"""Adapt the certified common-time plan to the existing per-vehicle trackers."""

from hashlib import sha256
from math import atan2, ceil, cos, hypot, sin

from builtin_interfaces.msg import Duration
from fwis_backends.multi_4wis.formation_continuous_a import contracts
from fwis_backends.multi_4wis.formation_playback import (
    PLAYBACK_PLAN_BACKEND, fields_from_timed_message, verify_uniform_playback_plan,
)
from fwis_backends.multi_4wis.formation_request import validate_formation_timed_trajectory
from fwis_backends.multi_4wis.uniform_timing import SetupTiming, legacy_mode_word
from fwis_control.model import wrap
from fwis_control.ros_state import stamp_ns
from fwis_v3_msgs.msg import TimedPoint, TimedTrajectory
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header


PERMIT_TOPIC = "/formation/execution_permit"


def pose_tuple(pose):
    return pose.x, pose.y, pose.yaw


def fit_reference(offsets, positions):
    """Rigid 2D least-squares fit from named vehicle centres to the wedge."""
    n = len(offsets)
    ox = sum(p[0] for p in offsets) / n
    oy = sum(p[1] for p in offsets) / n
    x = sum(p[0] for p in positions) / n
    y = sum(p[1] for p in positions) / n
    dot = cross = 0.
    for (rx, ry), (px, py, _) in zip(offsets, positions, strict=True):
        dot += (rx - ox) * (px - x) + (ry - oy) * (py - y)
        cross += (rx - ox) * (py - y) - (ry - oy) * (px - x)
    heading = atan2(cross, dot)
    c, s = cos(heading), sin(heading)
    return x - c * ox + s * oy, y - s * ox - c * oy, heading


def pose_matches(point, reference, position_tolerance, yaw_tolerance):
    return (hypot(point.x - reference[0], point.y - reference[1]) <= position_tolerance
            and abs(wrap(point.theta - reference[2])) <= yaw_tolerance)


def uniform_control_points(model):
    """Project the verified model onto one phase-aligned tracker time grid."""
    tracks = {vehicle_id: [] for vehicle_id in model.vehicle_ids}

    def append(time, requested_mode=None):
        state = model.state_at(time)
        mode = legacy_mode_word(state.mode if requested_mode is None else requested_mode)
        sec, nanosec = divmod(round(time * 1_000_000_000), 1_000_000_000)
        stamp = Duration(sec=sec, nanosec=nanosec)
        for vehicle in state.vehicles:
            speed = (vehicle.longitudinal_speed_mps if mode == TimedPoint.ACKERMANN
                     else vehicle.center_speed_mps)
            pose = Pose2D(x=vehicle.x, y=vehicle.y, theta=vehicle.theta)
            tracks[vehicle.vehicle_id].append(TimedPoint(
                time_from_start=stamp, center_pose=pose, v=speed, mode=mode))

    for phase in model.phases:
        count = ceil(phase.duration_s / .05)
        for index in range(count):
            time = phase.t0_s + phase.duration_s * (index / count)
            # The first interval of a stopped setup keeps the incoming mode.
            # Request the target mode while still stopped, with the rest of the
            # certified setup reserved for confirmation and settling.
            requested = phase.to_mode if index and isinstance(phase, SetupTiming) else phase.mode
            append(time, requested)
    append(model.duration_s)
    return tracks


def make_routes(envelope, request, start, *, expected_backend=None):
    timed = envelope.trajectory
    if (envelope.header.frame_id != "map" or timed.header != envelope.header
            or stamp_ns(envelope.header.stamp) <= 0 or envelope.plan_id != request.plan_id
            or timed.plan_id != envelope.plan_id or envelope.formation_id != request.formation_id
            or timed.formation_id != request.formation_id):
        raise ValueError("formation envelope identity, frame or stamp mismatch")
    if expected_backend is not None and envelope.backend != expected_backend:
        raise ValueError("formation envelope backend differs from the configured planner")
    result = validate_formation_timed_trajectory(timed)
    if not result.valid:
        raise ValueError(f"invalid synchronized trajectory {result.field}: {result.detail}")
    if len(timed.times_from_start) < 2 or stamp_ns(timed.times_from_start[0]) != 0:
        raise ValueError("formation trajectory requires a common zero and a terminal sample")
    expected = {spec.vehicle_id: spec for spec in request.vehicles}
    if set(result.snapshot.vehicle_ids) != set(expected):
        raise ValueError("formation vehicle IDs differ from the admitted request")
    for actual in timed.vehicles:
        wanted = expected[actual.vehicle_id]
        if actual.offset_in_formation != wanted.offset_in_formation:
            raise ValueError(f"formation offset mismatch for robot{actual.vehicle_id}")
    for point, target in ((timed.formation_poses[0], request.formation_start),
                          (timed.formation_poses[-1], request.formation_goal)):
        if not pose_matches(point, (target.x, target.y, target.theta), .02, .02):
            raise ValueError("formation reference endpoints differ from the request")

    projected = None
    if envelope.backend == PLAYBACK_PLAN_BACKEND:
        _, model, _ = verify_uniform_playback_plan(
            plan_id=envelope.plan_id, formation_id=envelope.formation_id, backend=envelope.backend,
            semantic_version=envelope.semantic_version, model_version=envelope.model_version,
            model_sha256=envelope.model_sha256, model_json=envelope.model_json,
            certification=envelope.certification, trajectory_fields=fields_from_timed_message(timed))
        projected = uniform_control_points(model)
    elif (envelope.backend != contracts.BACKEND or envelope.certification != contracts.PLAYBACK_CERTIFICATION
          or envelope.semantic_version != contracts.PLAYBACK_SEMANTIC_VERSION
          or envelope.model_version != contracts.TIMED_MODEL_VERSION or not envelope.model_json
          or envelope.model_sha256 != sha256(envelope.model_json.encode("utf-8")).hexdigest()):
        raise ValueError("formation envelope lacks the expected motion certification")

    routes = {}
    for track in timed.trajectories:
        wanted = expected[track.vehicle_id].start_center
        if not pose_matches(track.points[0].center_pose, (wanted.x, wanted.y, wanted.theta), .02, .02):
            raise ValueError(f"route starts away from robot{track.vehicle_id}")
        if projected is None:
            if any(p.mode != TimedPoint.ACKERMANN for p in track.points):
                raise ValueError(f"virtual Ackermann certification has a mode change for robot{track.vehicle_id}")
            points = [TimedPoint(time_from_start=p.time_from_start, center_pose=p.center_pose,
                                 v=p.center_speed_mps, mode=p.mode) for p in track.points]
        else:
            points = projected[track.vehicle_id]
        routes[track.vehicle_id] = TimedTrajectory(
            header=Header(frame_id="map", stamp=start), plan_id=envelope.plan_id, points=points)
    return routes
