"""FR05: atomic V0.2 playback envelope and analytic display frame (no ROS).

The envelope embeds one complete ``UniformTimedPlan`` model plus the legacy
``FormationTimedTrajectory`` sample carrier.  These tests prove that the
receiver accepts only envelopes whose model, digest, identity, metadata and
every published sample agree, and that playback derives the display frame
from the FR02 analytic sampler rather than from linear interpolation of the
published points.

Run from the V0.2 worktree root::

    OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.test_uniform_playback_plan -v
"""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.formation import (
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_playback import (
    PLAYBACK_PLAN_BACKEND, PLAYBACK_PLAN_SCHEMA, HOLD_END, HOLD_START,
    PLAYING, PlaybackValidationError,
    build_markers, build_uniform_sweep_markers, fields_from_timed_message,
    frame_at, status_document,
    trajectory_from_fields, verify_uniform_playback_plan)
from fwis_v3_optimizer.uniform_motion import (
    a_arc_edge, a_arc_required_headings, a_straight_edge,
    o_translation_edge, s_spin_edge)
from fwis_v3_optimizer.uniform_trajectory import (
    CERTIFICATION, MODEL_VERSION, UniformTimedPlan)

OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
BODY_SHAPES = {vehicle_id: VehicleShape(0.738, 0.500)
               for vehicle_id, _, _ in OFFSETS}
VEHICLE_IDS = tuple(vehicle_id for vehicle_id, _, _ in OFFSETS)
STAMP = (1700000000, 250000000)


def model():
    return FormationModel(tuple(
        VehicleOffset(vehicle_id, x, y) for vehicle_id, x, y in OFFSETS))


def directions(value=1):
    return {vehicle_id: int(value) for vehicle_id in VEHICLE_IDS}


def planar_plan():
    """S alignment -> non-zero-pivot A arc -> O translation.

    The cargo passes through a pivot at ``(0, 2)`` in the cargo frame, so the
    vehicle centres follow circular arcs rather than straight chords between
    published samples.
    """
    qf = FormationPose(2.0, -1.0, 0.35)
    start_headings = {1: 0.10, 2: -0.20, 3: 0.30}
    delta_psi = 0.60
    pivot = (0.0, 2.0)
    arc_headings = a_arc_required_headings(
        model(), qf, delta_psi, pivot, directions())
    alignment = s_spin_edge(
        model(), qf, start_headings,
        {vehicle_id: arc_headings[vehicle_id] - start_headings[vehicle_id]
         for vehicle_id in start_headings})
    arc = a_arc_edge(
        model(), qf, dict(alignment.end_headings), delta_psi, pivot,
        directions())
    translation_angle = math.atan2(0.70, 0.45)
    translation_headings = dict(arc.end_headings)
    translation = o_translation_edge(
        model(), arc.end_qf, translation_headings, (0.45, 0.70),
        directions(),
        {vehicle_id: translation_angle - translation_headings[vehicle_id]
         for vehicle_id in translation_headings})
    return UniformTimedPlan.from_edges(
        (alignment, arc, translation), BODY_SHAPES,
        plan_id=700123, formation_id="fr05_three_vehicle")


def straight_plan(plan_id=700124, distance_m=1.5):
    edge = a_straight_edge(
        model(), FormationPose(10.0, 10.0, 0.0),
        {vehicle_id: 0.0 for vehicle_id in VEHICLE_IDS}, (distance_m, 0.0),
        directions())
    return UniformTimedPlan.from_edges(
        (edge,), BODY_SHAPES, plan_id=plan_id,
        formation_id="fr05_three_vehicle")


def hold_plan():
    start = FormationPose(10.0, 10.0, 0.0)
    headings = {vehicle_id: 0.0 for vehicle_id in VEHICLE_IDS}
    return UniformTimedPlan.from_edges(
        (), BODY_SHAPES, plan_id=700125,
        formation_id="fr05_three_vehicle", model=model(),
        start_qf=start, start_headings=headings,
        expected_end_qf=start, expected_end_headings=headings)


def envelope_fields(plan, stamp=STAMP):
    """Build the wire fields the uniform planner publishes for ``plan``."""
    projection = plan.sample()
    timed = projection.timed_trajectory
    start_centers = plan.model.centers(plan.start_qf)
    goal_centers = plan.model.centers(plan.goal_qf)
    start_headings = dict(plan.start_headings)
    goal_headings = dict(plan.goal_headings)
    vehicles = []
    for offset in plan.model.offsets:
        shape = plan.vehicle_shapes[offset.vehicle_id]
        vehicles.append({
            "vehicle_id": offset.vehicle_id,
            "offset": (offset.x, offset.y),
            "body_length_m": shape.length_m,
            "body_width_m": shape.width_m,
            "start_center": (start_centers[offset.vehicle_id][0],
                             start_centers[offset.vehicle_id][1],
                             start_headings[offset.vehicle_id]),
            "goal_center": (goal_centers[offset.vehicle_id][0],
                            goal_centers[offset.vehicle_id][1],
                            goal_headings[offset.vehicle_id]),
            "goal_heading_free": False,
        })
    trajectories = []
    for vehicle_id in sorted(timed.trajectories):
        points = tuple(
            (point.time_from_start, point.center_x, point.center_y,
             point.theta, point.center_speed_mps,
             point.center_yaw_rate_radps, int(point.mode),
             point.mode_control)
            for point in timed.trajectories[vehicle_id])
        trajectories.append({"vehicle_id": vehicle_id, "points": points})
    return {
        "frame_id": "map",
        "stamp": (int(stamp[0]), int(stamp[1])),
        "plan_id": plan.plan_id,
        "formation_id": plan.formation_id,
        "vehicles": tuple(vehicles),
        "formation_poses": tuple(
            (pose.x, pose.y, pose.psi) for pose in timed.formation_poses),
        "times_from_start": tuple(timed.times_from_start),
        "trajectories": tuple(trajectories),
    }


def verify(plan, fields=None, **overrides):
    payload = {
        "plan_id": plan.plan_id,
        "formation_id": plan.formation_id,
        "backend": PLAYBACK_PLAN_BACKEND,
        "semantic_version": PLAYBACK_PLAN_SCHEMA,
        "model_version": MODEL_VERSION,
        "model_sha256": plan.model_sha256(),
        "model_json": plan.to_model_json(),
        "certification": CERTIFICATION,
        "trajectory_fields": envelope_fields(plan)
        if fields is None else fields,
    }
    payload.update(overrides)
    return verify_uniform_playback_plan(**payload)


class _FakeMessage(object):
    """Minimal stand-in for FormationTimedTrajectory used without ROS."""

    class Header(object):
        def __init__(self, frame_id, stamp):
            self.frame_id = frame_id
            self.stamp = type("Stamp", (), {
                "secs": stamp[0], "nsecs": stamp[1]})()

    def __init__(self, fields):
        self.header = self.Header(fields["frame_id"], fields["stamp"])
        self.plan_id = fields["plan_id"]
        self.formation_id = fields["formation_id"]
        self.vehicles = [type("Vehicle", (), {
            "vehicle_id": item["vehicle_id"],
            "offset_in_formation": type("Offset", (), {
                "x": item["offset"][0], "y": item["offset"][1]})(),
            "body_length_m": item["body_length_m"],
            "body_width_m": item["body_width_m"],
            "start_center": type("Center", (), {
                "x": item["start_center"][0],
                "y": item["start_center"][1],
                "theta": item["start_center"][2]})(),
            "goal_center": type("Center", (), {
                "x": item["goal_center"][0],
                "y": item["goal_center"][1],
                "theta": item["goal_center"][2]})(),
            "goal_heading_free": item["goal_heading_free"],
        })() for item in fields["vehicles"]]
        self.formation_poses = [type("Pose", (), {
            "x": pose[0], "y": pose[1], "theta": pose[2]})()
            for pose in fields["formation_poses"]]
        self.times_from_start = [
            type("Duration", (), {"to_sec": lambda self, value=value: value})()
            for value in fields["times_from_start"]]
        self.trajectories = [
            type("Trajectory", (), {
                "vehicle_id": item["vehicle_id"],
                "points": [type("Point", (), {
                    "time_from_start": type(
                        "Duration", (),
                        {"to_sec": lambda self, value=point[0]: value})(),
                    "center_pose": type("Pose", (), {
                        "x": point[1], "y": point[2], "theta": point[3]})(),
                    "center_speed_mps": point[4],
                    "center_yaw_rate_radps": point[5],
                    "mode": point[6],
                    "mode_control": point[7]})()
                    for point in item["points"]]})()
            for item in fields["trajectories"]]


class EnvelopeAcceptanceTest(unittest.TestCase):
    maxDiff = None

    def test_valid_envelope_rebuilds_the_authoritative_model(self):
        plan = planar_plan()
        trajectory, rebuilt, digest = verify(plan)
        self.assertEqual(rebuilt.plan_id, plan.plan_id)
        self.assertEqual(rebuilt.formation_id, plan.formation_id)
        self.assertEqual(digest, plan.model_sha256())
        self.assertEqual(trajectory.plan_id, plan.plan_id)
        self.assertIsNotNone(trajectory.uniform_plan)
        self.assertEqual(trajectory.uniform_plan.model_sha256(), digest)
        self.assertGreaterEqual(trajectory.sample_count, 4)

    def test_fields_round_trip_through_the_message_converter(self):
        fields = envelope_fields(planar_plan())
        message = _FakeMessage(fields)
        self.assertEqual(fields_from_timed_message(message), fields)

    def test_zero_motion_hold_envelope_is_accepted(self):
        plan = hold_plan()
        trajectory, _, _ = verify(plan)
        self.assertEqual(trajectory.duration, 0.0)

    def test_unknown_backend_is_rejected(self):
        with self.assertRaises(PlaybackValidationError):
            verify(planar_plan(), backend="legacy")

    def test_unknown_semantic_or_model_version_is_rejected(self):
        plan = planar_plan()
        with self.assertRaises(PlaybackValidationError):
            verify(plan, semantic_version="uniform_playback_plan/v0")
        with self.assertRaises(PlaybackValidationError):
            verify(plan, model_version="uniform_timed_plan.v0")

    def test_digest_mismatch_is_rejected(self):
        plan = planar_plan()
        digest = plan.model_sha256()
        flipped = ("0" if digest[0] != "0" else "1") + digest[1:]
        with self.assertRaisesRegex(PlaybackValidationError, "sha256"):
            verify(plan, model_sha256=flipped)

    def test_tampered_model_json_is_rejected(self):
        """A model that no longer matches its own edges cannot be rebuilt."""
        plan = planar_plan()
        document = json.loads(plan.to_model_json())
        document["requested_goal"]["qf"][0] += 0.25
        with self.assertRaises(PlaybackValidationError):
            verify(plan, model_json=json.dumps(document))

    def test_rebuildable_but_different_model_is_rejected_by_digest(self):
        """A plausible model with the original digest still fails closed."""
        plan = planar_plan()
        other = straight_plan(plan_id=plan.plan_id)
        with self.assertRaisesRegex(PlaybackValidationError, "sha256"):
            verify(plan, model_json=other.to_model_json())

    def test_tampered_sample_is_rejected(self):
        plan = planar_plan()
        fields = envelope_fields(plan)
        trajectories = [dict(item) for item in fields["trajectories"]]
        index = len(trajectories[0]["points"]) // 2
        point = list(trajectories[0]["points"][index])
        point[1] += 1e-3
        trajectories[0]["points"] = (
            trajectories[0]["points"][:index] + (tuple(point),)
            + trajectories[0]["points"][index + 1:])
        fields = dict(fields, trajectories=tuple(trajectories))
        with self.assertRaisesRegex(PlaybackValidationError, "sample"):
            verify(plan, fields=fields)

    def test_tampered_timing_is_rejected(self):
        plan = planar_plan()
        fields = envelope_fields(plan)
        times = list(fields["times_from_start"])
        times[1] += 1e-3
        with self.assertRaises(PlaybackValidationError):
            verify(plan, fields=dict(fields, times_from_start=tuple(times)))

    def test_terminal_timestamp_can_round_up_to_ros_tick(self):
        plan = straight_plan(distance_m=1.25)
        fields = envelope_fields(plan)
        secs, nsecs = divmod(round(plan.duration_s * 1e9), 1000000000)
        wire_end = secs + nsecs / 1e9
        self.assertGreater(wire_end, plan.duration_s)

        def set_terminal_time(time_s):
            fields["times_from_start"] = fields["times_from_start"][:-1] + (time_s,)
            for vehicle in fields["trajectories"]:
                points = vehicle["points"]
                vehicle["points"] = points[:-1] + ((time_s,) + points[-1][1:],)

        set_terminal_time(wire_end)
        verified, _, _ = verify(plan, fields=fields)
        self.assertEqual(verified.duration, wire_end)
        held = frame_at(verified, wire_end + 1.0)
        self.assertEqual(held.state, HOLD_END)
        self.assertEqual(held.formation_x, plan.goal_qf.x)
        self.assertTrue(all(vehicle.center_speed_mps == 0.0
                            for vehicle in held.vehicles))
        set_terminal_time(wire_end + 2e-9)
        with self.assertRaisesRegex(PlaybackValidationError, "trajectory duration"):
            verify(plan, fields=fields)

    def test_metadata_mismatches_are_rejected(self):
        plan = planar_plan()
        base = envelope_fields(plan)

        def mutation(name, apply):
            fields = copy.deepcopy(base)
            apply(fields)
            with self.subTest(name=name):
                with self.assertRaises(PlaybackValidationError):
                    verify(plan, fields=fields)

        def shift_start(fields):
            vehicle = dict(fields["vehicles"][0])
            vehicle["start_center"] = (
                vehicle["start_center"][0] + 0.2,) + (
                vehicle["start_center"][1:])
            fields["vehicles"] = (vehicle,) + fields["vehicles"][1:]

        def shift_goal(fields):
            vehicle = dict(fields["vehicles"][1])
            vehicle["goal_center"] = (
                vehicle["goal_center"][0],
                vehicle["goal_center"][1] + 0.2,
                vehicle["goal_center"][2])
            fields["vehicles"] = (
                fields["vehicles"][0], vehicle) + fields["vehicles"][2:]

        def wrong_offset(fields):
            vehicle = dict(fields["vehicles"][2])
            vehicle["offset"] = (vehicle["offset"][0], 1.25)
            fields["vehicles"] = fields["vehicles"][:2] + (vehicle,)

        def wrong_shape(fields):
            vehicle = dict(fields["vehicles"][0])
            vehicle["body_width_m"] = 0.75
            fields["vehicles"] = (vehicle,) + fields["vehicles"][1:]

        def wrong_endpoint(fields):
            poses = list(fields["formation_poses"])
            poses[-1] = (poses[-1][0] + 0.1,) + poses[-1][1:]
            fields["formation_poses"] = tuple(poses)

        def wrong_mode(fields):
            item = dict(fields["trajectories"][0])
            points = list(item["points"])
            point = list(points[0])
            point[6] = 2
            points[0] = tuple(point)
            item["points"] = tuple(points)
            fields["trajectories"] = (item,) + fields["trajectories"][1:]

        mutation("start center", shift_start)
        mutation("goal center", shift_goal)
        mutation("offset", wrong_offset)
        mutation("body shape", wrong_shape)
        mutation("formation endpoint", wrong_endpoint)
        mutation("mode word", wrong_mode)

    def test_identity_mismatch_is_rejected(self):
        plan = planar_plan()
        with self.assertRaises(PlaybackValidationError):
            verify(plan, plan_id=plan.plan_id + 1)
        with self.assertRaises(PlaybackValidationError):
            verify(plan, formation_id="other_formation")

    def test_certification_mismatch_is_rejected(self):
        plan = planar_plan()
        with self.assertRaises(PlaybackValidationError):
            verify(plan, certification="CERTIFIED_AND_SAFE")


class AnalyticFrameTest(unittest.TestCase):

    def test_straight_sweep_covers_the_full_vehicle_footprint(self):
        trajectory, _, _ = verify(straight_plan())
        markers = build_uniform_sweep_markers(trajectory)
        fills = [marker for marker in markers if marker.marker_type == "TRIANGLE_LIST"]
        self.assertEqual(len(fills), 3)
        for marker, spec in zip(fills, trajectory.vehicles):
            xs, ys, _ = zip(*marker.points)
            self.assertAlmostEqual(max(xs) - min(xs), 1.5 + spec.body_length_m)
            self.assertAlmostEqual(max(ys) - min(ys), spec.body_width_m)

    def test_frame_matches_the_model_and_not_linear_interpolation(self):
        plan = planar_plan()
        trajectory, _, _ = verify(plan)
        times = trajectory.times_from_start
        # Choose an interior instant strictly between two published samples.
        index = next(i for i in range(1, len(times) - 1)
                     if times[i + 1] - times[i] > 0.02)
        elapsed = 0.5 * (times[index] + times[index + 1])
        frame = frame_at(trajectory, elapsed)
        self.assertEqual(frame.state, "PLAYING")
        state = plan.state_at(elapsed)
        self.assertAlmostEqual(frame.formation_x, state.qf.x, places=12)
        self.assertAlmostEqual(frame.formation_y, state.qf.y, places=12)
        self.assertAlmostEqual(frame.formation_psi, state.qf.psi, places=12)
        centres = plan.model.centers(state.qf)
        worst = 0.0
        for vehicle in frame.vehicles:
            expected = centres[vehicle.vehicle_id]
            worst = max(worst,
                        abs(vehicle.center_x - expected[0]),
                        abs(vehicle.center_y - expected[1]))
        self.assertLessEqual(worst, 1e-12)

        # Linear interpolation of the two neighbouring samples would move the
        # non-zero-pivot arc by millimetres; the analytic frame must not.
        linear_x = 0.5 * (
            trajectory.points[vehicle.vehicle_id][index].center_x
            + trajectory.points[vehicle.vehicle_id][index + 1].center_x)
        sample = trajectory.points[vehicle.vehicle_id][index]
        later = trajectory.points[vehicle.vehicle_id][index + 1]
        linear_y = 0.5 * (sample.center_y + later.center_y)
        vehicle = frame.vehicles[0]
        interpolation_error = math.hypot(
            vehicle.center_x - linear_x, vehicle.center_y - linear_y)
        self.assertGreater(interpolation_error, 5e-4)

    def test_frame_hold_states_and_clamping(self):
        plan = planar_plan()
        trajectory, _, _ = verify(plan)
        before = frame_at(trajectory, -1.0)
        self.assertEqual(before.state, HOLD_START)
        self.assertEqual(before.clamped_elapsed_s, 0.0)
        after = frame_at(trajectory, trajectory.duration + 2.0)
        self.assertEqual(after.state, HOLD_END)
        self.assertEqual(after.clamped_elapsed_s, trajectory.duration)

    def test_forward_plan_frame_uses_the_sampler(self):
        plan = straight_plan()
        trajectory, _, _ = verify(plan)
        midpoint = 0.5 * trajectory.duration
        frame = frame_at(trajectory, midpoint)
        state = plan.state_at(midpoint)
        self.assertAlmostEqual(frame.formation_x, state.qf.x, places=12)
        self.assertAlmostEqual(frame.formation_y, state.qf.y, places=12)

    def test_status_document_carries_the_envelope_identity(self):
        plan = planar_plan()
        trajectory, _, digest = verify(plan)
        frame = frame_at(trajectory, 0.0)
        document = status_document(
            trajectory, frame, status="FEASIBLE",
            backend=PLAYBACK_PLAN_BACKEND,
            semantic_version=PLAYBACK_PLAN_SCHEMA,
            model_sha256=digest)
        self.assertEqual(document["backend"], PLAYBACK_PLAN_BACKEND)
        self.assertEqual(document["semantic_version"], PLAYBACK_PLAN_SCHEMA)
        self.assertEqual(document["model_sha256"], digest)
        cleared = status_document(
            cleared=True, backend=PLAYBACK_PLAN_BACKEND,
            semantic_version=PLAYBACK_PLAN_SCHEMA,
            model_sha256=digest)
        self.assertTrue(cleared["cleared"])
        self.assertEqual(cleared["backend"], PLAYBACK_PLAN_BACKEND)


class AuditIdentityBindingTest(unittest.TestCase):
    """FR05-R2: the display label is bound to the verified model digest.

    A status that merely shares ``plan_id`` with the display is not an audit
    of the displayed model.  Only the same plan_id, formation_id, backend and
    model_sha256 may upgrade the display to FEASIBLE/executable.
    """

    @staticmethod
    def _feasible_status(plan, digest, **overrides):
        status = {
            "plan_id": plan.plan_id,
            "formation_id": plan.formation_id,
            "backend": PLAYBACK_PLAN_BACKEND,
            "model_sha256": digest,
            "status": "FEASIBLE",
            "feasible": True,
            "executable": True,
            "trajectory_published": True,
        }
        status.update(overrides)
        return status

    def test_same_plan_id_other_digest_never_certifies_the_display(self):
        foreign_plan = straight_plan(distance_m=1.5)
        display_plan = straight_plan(distance_m=2.0)
        self.assertEqual(foreign_plan.plan_id, display_plan.plan_id)
        self.assertNotEqual(foreign_plan.model_sha256(),
                            display_plan.model_sha256())
        trajectory, _, digest = verify(display_plan)
        frame = frame_at(trajectory, 0.5 * trajectory.duration)
        foreign = self._feasible_status(
            foreign_plan, foreign_plan.model_sha256())
        document = status_document(
            trajectory, frame, foreign, model_sha256=digest,
            backend=PLAYBACK_PLAN_BACKEND)
        markers, _, audit, executable = build_markers(
            trajectory, frame, status=foreign)
        self.assertFalse(document["executable"])
        self.assertEqual(document["certification"], "GEOMETRY_ONLY")
        self.assertFalse(executable)
        self.assertEqual(audit, "GEOMETRY_ONLY")

    def test_missing_identity_fields_never_certify_the_display(self):
        plan = straight_plan()
        trajectory, _, digest = verify(plan)
        frame = frame_at(trajectory, 0.5 * trajectory.duration)
        for missing in ("model_sha256", "formation_id", "backend"):
            status = self._feasible_status(plan, digest)
            del status[missing]
            document = status_document(
                trajectory, frame, status, model_sha256=digest,
                backend=PLAYBACK_PLAN_BACKEND)
            markers, _, audit, executable = build_markers(
                trajectory, frame, status=status)
            self.assertFalse(document["executable"], missing)
            self.assertFalse(executable, missing)
            self.assertEqual(audit, "GEOMETRY_ONLY", missing)

    def test_matching_identity_certifies_the_display(self):
        plan = straight_plan()
        trajectory, _, digest = verify(plan)
        self.assertEqual(trajectory.model_sha256, digest)
        frame = frame_at(trajectory, 0.5 * trajectory.duration)
        status = self._feasible_status(plan, digest)
        document = status_document(
            trajectory, frame, status, model_sha256=digest,
            backend=PLAYBACK_PLAN_BACKEND)
        markers, _, audit, executable = build_markers(
            trajectory, frame, status=status)
        self.assertTrue(document["executable"])
        self.assertEqual(document["certification"], "FEASIBLE")
        self.assertTrue(executable)
        self.assertEqual(audit, "FEASIBLE")

    def test_status_document_rejects_a_digest_for_another_envelope(self):
        plan = straight_plan()
        trajectory, _, digest = verify(plan)
        frame = frame_at(trajectory, 0.5 * trajectory.duration)
        with self.assertRaises(PlaybackValidationError):
            status_document(
                trajectory, frame,
                self._feasible_status(plan, digest),
                model_sha256="0" * 64, backend=PLAYBACK_PLAN_BACKEND)

    def test_verified_envelope_carries_the_authoritative_digest(self):
        plan = straight_plan()
        trajectory, rebuilt, digest = verify(plan)
        self.assertEqual(rebuilt.model_sha256(), digest)
        self.assertEqual(trajectory.model_sha256, digest)

    def test_zero_motion_hold_still_certifies_with_matching_identity(self):
        plan = hold_plan()
        trajectory, _, digest = verify(plan)
        status = self._feasible_status(plan, digest, status="NO_MOTION")
        frame = frame_at(trajectory, 0.0)
        document = status_document(
            trajectory, frame, status, model_sha256=digest,
            backend=PLAYBACK_PLAN_BACKEND)
        self.assertTrue(document["executable"])
        self.assertEqual(document["certification"], "NO_MOTION")

    def test_legacy_trajectory_keeps_the_plan_id_only_audit(self):
        plan = straight_plan()
        legacy = trajectory_from_fields(envelope_fields(plan))
        self.assertIsNone(legacy.model_sha256)
        frame = frame_at(legacy, 0.5 * legacy.duration)
        status = self._feasible_status(plan, "0" * 64)
        document = status_document(legacy, frame, status)
        markers, _, audit, executable = build_markers(
            legacy, frame, status=status)
        self.assertTrue(document["executable"])
        self.assertTrue(executable)
        self.assertEqual(audit, "FEASIBLE")


class TrajectoryFieldValidationTest(unittest.TestCase):

    def test_short_point_tuple_is_rejected(self):
        fields = envelope_fields(straight_plan())
        item = dict(fields["trajectories"][0])
        item["points"] = tuple(tuple(point[:6]) for point in item["points"])
        fields = dict(fields, trajectories=(item,) + fields["trajectories"][1:])
        with self.assertRaises(PlaybackValidationError):
            trajectory_from_fields(fields)

    def test_non_increasing_shared_clock_is_rejected(self):
        fields = envelope_fields(straight_plan())
        times = list(fields["times_from_start"])
        self.assertGreaterEqual(len(times), 2)
        times[1] = times[0]
        with self.assertRaises(PlaybackValidationError):
            trajectory_from_fields(dict(fields, times_from_start=tuple(times)))


if __name__ == "__main__":
    unittest.main()
