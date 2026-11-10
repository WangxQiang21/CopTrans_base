"""S3 contracts for formation request validation and ROS message freeze."""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_request import (
    INVALID_REQUEST,
    INVALID_TIMED_TRAJECTORY,
    validate_formation_request,
    validate_formation_timed_trajectory,
)

MSG_DIR = REPO_ROOT / "catkin_ws/src/fwis_v3_msgs/msg"
DEVEL_PYTHON = REPO_ROOT / "catkin_ws/devel/lib/python3/dist-packages"
DEVEL_INCLUDE = REPO_ROOT / "catkin_ws/devel/include/fwis_v3_msgs"
ROS_PYTHON = Path("/opt/ros/noetic/lib/python3/dist-packages")

SINGLE_VEHICLE_MSG_SHA256 = {
    "ModePrimitive.msg":
        "7fa0345ae76c9f09055c9e17d898d8c79fec59aeb3c39549e911c9afb2d17b03",
    "PlanSeed.msg":
        "688a693eac0537271f07a40d0d3b42680d70d41cf6f3fe6b06c2d7248d0ad204",
    "PlanSeedCandidates.msg":
        "e76716dbc68f240a1660d3f011506524ad3e4ef00dfe04dc3b0d89e553031a43",
    "TimedPoint.msg":
        "55cf0349caad56a8122ce23fac470912e22ec676f723772ad9b3a24b8729ceb5",
    "TimedTrajectory.msg":
        "be55fdd47ca8d2f63f6759a4aa8549d2e93c92c1ebeedd990f54e2f405cb1c2e",
}
NEW_MSG_FILES = (
    "FormationVehicleSpec.msg",
    "FormationPlanRequest.msg",
    "FormationPrimitive.msg",
    "FormationPlanSeed.msg",
    "FormationTimedPoint.msg",
    "FormationVehicleTrajectory.msg",
    "FormationTimedTrajectory.msg",
)
NEW_MSG_TYPES = tuple(name[:-4] for name in NEW_MSG_FILES)


def _load_message_module():
    for path in (ROS_PYTHON, DEVEL_PYTHON):
        if path.is_dir() and str(path) not in sys.path:
            sys.path.insert(0, str(path))
    try:
        import fwis_v3_msgs.msg as message_module
    except ImportError:
        return None
    if any(not hasattr(message_module, name) for name in NEW_MSG_TYPES):
        return None
    return message_module


MESSAGE_MODULE = _load_message_module()

OFFSETS = ((0.0, 0.0), (-1.0, 0.0), (-1.0, -1.0))
QF_START = (2.0, 2.0, 0.35)
QF_GOAL = (6.0, 3.0, -0.4)
START_HEADINGS = (0.7, -0.3, 1.2)
GOAL_HEADINGS = (0.2, 2.0, -1.1)
FREE = (False, True, False)
BODY_LENGTH = 0.738
BODY_WIDTH = 0.5


def pose(x, y, theta):
    return SimpleNamespace(x=x, y=y, theta=theta)


def point(x, y, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def stamp(secs, nsecs=0):
    return SimpleNamespace(secs=secs, nsecs=nsecs)


def header(frame_id="map", secs=100, nsecs=5):
    return SimpleNamespace(frame_id=frame_id, stamp=stamp(secs, nsecs))


def rigid_center(qf, offset):
    x, y, psi = qf
    return (x + offset[0] * math.cos(psi) - offset[1] * math.sin(psi),
            y + offset[0] * math.sin(psi) + offset[1] * math.cos(psi))


def vehicle_spec(index, qf_start=QF_START, qf_goal=QF_GOAL,
                 goal_shift=(0.0, 0.0), start_shift=(0.0, 0.0),
                 heading_free=None, vehicle_id=None, **overrides):
    vehicle_id = index if vehicle_id is None else vehicle_id
    start = rigid_center(qf_start, OFFSETS[index])
    goal = rigid_center(qf_goal, OFFSETS[index])
    values = dict(
        vehicle_id=vehicle_id,
        offset_in_formation=point(*OFFSETS[index]),
        start_center=pose(start[0] + start_shift[0],
                          start[1] + start_shift[1], START_HEADINGS[index]),
        goal_center=pose(goal[0] + goal_shift[0],
                         goal[1] + goal_shift[1], GOAL_HEADINGS[index]),
        goal_heading_free=(FREE[index] if heading_free is None
                           else heading_free),
        body_length_m=BODY_LENGTH,
        body_width_m=BODY_WIDTH,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def formation_request(plan_id=17, formation_id="wedge_3", vehicles=None,
                      frame_id="map", stamp_secs=100, stamp_nsecs=5,
                      allow_reverse=False, qf_start=QF_START, qf_goal=QF_GOAL):
    if vehicles is None:
        vehicles = [vehicle_spec(index, qf_start, qf_goal)
                    for index in range(len(OFFSETS))]
    return SimpleNamespace(
        header=header(frame_id, stamp_secs, stamp_nsecs),
        plan_id=plan_id,
        formation_id=formation_id,
        formation_start=pose(*qf_start),
        formation_goal=pose(*qf_goal),
        vehicles=vehicles,
        allow_reverse=allow_reverse)


TRAJECTORY_TIMES = ((10, 0), (11, 0), (12, 0))


def timed_point(time, mode=0, speed=0.5, yaw_rate=0.0, control=0.0, theta=0.0):
    return SimpleNamespace(
        time_from_start=stamp(*time),
        center_pose=pose(4.0, 4.0, theta),
        center_speed_mps=speed,
        center_yaw_rate_radps=yaw_rate,
        mode=mode,
        mode_control=control)


def make_timed_trajectory(times=TRAJECTORY_TIMES, vehicle_ids=(0, 1),
                          trajectory_ids=None, points_lengths=None,
                          formation_pose_count=None, point_time_overrides=None,
                          frame_id="map", stamp_secs=100, stamp_nsecs=5):
    trajectory_ids = vehicle_ids if trajectory_ids is None else trajectory_ids
    specs = [vehicle_spec(index) for index in vehicle_ids]
    poses = [pose(4.0 + 0.1 * index, 4.0, 0.2)
             for index in range(len(times))]
    if formation_pose_count is not None:
        poses = poses[:formation_pose_count]
    trajectories = []
    for trajectory_index, vehicle_id in enumerate(trajectory_ids):
        lengths = (len(times) if points_lengths is None
                   else points_lengths[trajectory_index])
        points = []
        for point_index in range(lengths):
            time = times[point_index]
            override = (point_time_overrides or {}).get(
                (trajectory_index, point_index))
            if override is not None:
                time = override
            points.append(timed_point(time, theta=0.1 * vehicle_id))
        trajectories.append(SimpleNamespace(
            vehicle_id=vehicle_id, points=points))
    return SimpleNamespace(
        header=header(frame_id, stamp_secs, stamp_nsecs),
        plan_id=21,
        formation_id="wedge_2",
        vehicles=specs,
        formation_poses=poses,
        times_from_start=[stamp(*time) for time in times],
        trajectories=trajectories)


class FormationRequestContracts(unittest.TestCase):
    def test_valid_request_rebuilds_s1_s2_objects_and_both_ids(self):
        result = validate_formation_request(formation_request())
        self.assertTrue(result.valid, msg=result)
        snapshot = result.snapshot
        self.assertEqual(snapshot.plan_id, 17)
        self.assertEqual(snapshot.formation_id, "wedge_3")
        self.assertEqual(snapshot.header_stamp, (100, 5))
        self.assertEqual(snapshot.start, FormationPose(*QF_START))
        self.assertEqual(snapshot.goal, FormationPose(*QF_GOAL))
        self.assertEqual(snapshot.model.vehicle_ids, (0, 1, 2))
        self.assertEqual(tuple(snapshot.vehicle_shapes), (0, 1, 2))
        for vehicle_id in snapshot.vehicle_ids:
            self.assertEqual(snapshot.vehicle_shapes[vehicle_id].length_m,
                             BODY_LENGTH)
            self.assertEqual(snapshot.vehicle_shapes[vehicle_id].width_m,
                             BODY_WIDTH)
            self.assertEqual(snapshot.start_headings[vehicle_id],
                             START_HEADINGS[vehicle_id])
            self.assertEqual(snapshot.goal_headings[vehicle_id],
                             GOAL_HEADINGS[vehicle_id])
        self.assertEqual(snapshot.goal_heading_free, {0: False, 1: True, 2: False})
        self.assertFalse(snapshot.allow_reverse)
        status = result.as_status()
        self.assertTrue(status["valid"])
        self.assertFalse(status["search_started"])
        self.assertFalse(status["trajectory_published"])
        self.assertFalse(status["executable"])

    def test_goal_heading_free_never_relaxes_center_rigidity(self):
        vehicles = [vehicle_spec(index, heading_free=True)
                    for index in range(len(OFFSETS))]
        vehicles[1] = vehicle_spec(1, heading_free=True,
                                   goal_shift=(0.01, 0.0))
        result = validate_formation_request(formation_request(vehicles=vehicles))
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, INVALID_REQUEST)
        self.assertEqual(result.field, "vehicles[1].goal_center")
        self.assertEqual(result.vehicle_id, 1)
        self.assertIn("goal_heading_free", result.detail)

        repaired = [vehicle_spec(index, heading_free=True)
                    for index in range(len(OFFSETS))]
        result = validate_formation_request(formation_request(vehicles=repaired))
        self.assertTrue(result.valid, msg=result)
        self.assertEqual(result.snapshot.goal_heading_free,
                         {0: True, 1: True, 2: True})
        self.assertEqual(result.snapshot.goal_headings,
                         {0: GOAL_HEADINGS[0], 1: GOAL_HEADINGS[1],
                          2: GOAL_HEADINGS[2]})

    def test_start_and_goal_centers_enforce_fixed_offsets(self):
        shifted_start = [vehicle_spec(index) for index in range(3)]
        shifted_start[2] = vehicle_spec(2, start_shift=(0.02, -0.01))
        start = validate_formation_request(
            formation_request(vehicles=shifted_start))
        self.assertFalse(start.valid)
        self.assertEqual(start.field, "vehicles[2].start_center")
        self.assertEqual(start.vehicle_id, 2)

        shifted_goal = [vehicle_spec(index) for index in range(3)]
        shifted_goal[0] = vehicle_spec(0, goal_shift=(-0.03, 0.0))
        goal = validate_formation_request(
            formation_request(vehicles=shifted_goal))
        self.assertFalse(goal.valid)
        self.assertEqual(goal.field, "vehicles[0].goal_center")
        self.assertEqual(goal.vehicle_id, 0)

    def test_mirrored_goal_positions_are_rejected_against_rigid_model(self):
        goal_centers = [rigid_center(QF_GOAL, offset) for offset in OFFSETS]
        vehicles = []
        for index, (center, offset) in enumerate(zip(goal_centers, OFFSETS)):
            spec = vehicle_spec(index)
            spec.goal_center = pose(center[0], -center[1],
                                    GOAL_HEADINGS[index])
            vehicles.append(spec)
        result = validate_formation_request(
            formation_request(vehicles=vehicles))
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, INVALID_REQUEST)
        self.assertEqual(result.field, "vehicles[0].goal_center")

    def test_nonpositive_goal_tolerance_is_rejected(self):
        request = formation_request()
        for tolerance in (-1.0, math.nan, math.inf, "bad"):
            with self.subTest(tolerance=tolerance):
                result = validate_formation_request(request, tolerance)
                self.assertFalse(result.valid)
                self.assertEqual(result.field, "position_tolerance_m")

    def test_duplicate_empty_and_bad_vehicle_ids_fail(self):
        empty = validate_formation_request(formation_request(vehicles=[]))
        self.assertFalse(empty.valid)
        self.assertEqual(empty.reason, INVALID_REQUEST)
        self.assertEqual(empty.field, "vehicles")
        self.assertIn("empty", empty.detail)

        duplicate = [vehicle_spec(index) for index in range(3)]
        duplicate.append(vehicle_spec(0))
        result = validate_formation_request(
            formation_request(vehicles=duplicate))
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "vehicles[3].vehicle_id")
        self.assertEqual(result.vehicle_id, 0)

        for bad_id in (True, -1, 2**32, 1.5):
            with self.subTest(vehicle_id=bad_id):
                result = validate_formation_request(formation_request(
                    vehicles=[vehicle_spec(0, vehicle_id=bad_id)]))
                self.assertFalse(result.valid)
                self.assertEqual(result.field, "vehicles[0].vehicle_id")

    def test_coincident_offsets_are_rejected_by_rebuilt_model(self):
        vehicles = [vehicle_spec(index) for index in range(3)]
        vehicles[2].offset_in_formation = point(*OFFSETS[0])
        result = validate_formation_request(
            formation_request(vehicles=vehicles))
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, INVALID_REQUEST)
        self.assertEqual(result.field, "vehicles.offset_in_formation")
        self.assertIn("COINCIDENT_OFFSETS", result.detail)

    def test_plan_id_and_formation_id_remain_distinct_fields(self):
        result = validate_formation_request(
            formation_request(plan_id=99, formation_id="wedge_3"))
        self.assertTrue(result.valid)
        self.assertEqual(result.snapshot.plan_id, 99)
        self.assertEqual(result.snapshot.formation_id, "wedge_3")
        status = result.as_status()
        self.assertNotIn("formation_id_as_plan_id", status)

        for plan_id in (-1, 2**32, True, 1.5):
            with self.subTest(plan_id=plan_id):
                result = validate_formation_request(
                    formation_request(plan_id=plan_id))
                self.assertFalse(result.valid)
                self.assertEqual(result.field, "plan_id")
        for formation_id in ("", "   ", 7, None):
            with self.subTest(formation_id=formation_id):
                result = validate_formation_request(
                    formation_request(formation_id=formation_id))
                self.assertFalse(result.valid)
                self.assertEqual(result.field, "formation_id")

    def test_header_and_spec_fields_are_strict(self):
        cases = (
            ({"frame_id": "odom"}, "header.frame_id"),
            ({"stamp_secs": 0, "stamp_nsecs": 0}, "header.stamp"),
            ({"stamp_nsecs": 1000000000}, "header.stamp"),
            ({"allow_reverse": 1}, "allow_reverse"),
        )
        for kwargs, field in cases:
            with self.subTest(kwargs=kwargs):
                result = validate_formation_request(
                    formation_request(**kwargs))
                self.assertFalse(result.valid)
                self.assertEqual(result.field, field)

        bad_specs = (
            ({"offset_in_formation": point(0.0, 0.0, 0.1)},
             "vehicles[0].offset_in_formation.z"),
            ({"offset_in_formation": point(math.nan, 0.0)},
             "vehicles[0].offset_in_formation.x"),
            ({"start_center": pose(math.inf, 0.0, 0.0)},
             "vehicles[0].start_center.x"),
            ({"start_center": pose(0.0, 0.0, math.nan)},
             "vehicles[0].start_center.theta"),
            ({"body_length_m": 0.0}, "vehicles[0].body_length_m"),
            ({"body_width_m": -0.5}, "vehicles[0].body_width_m"),
        )
        for overrides, field in bad_specs:
            with self.subTest(field=field):
                result = validate_formation_request(formation_request(
                    vehicles=[vehicle_spec(0, **overrides)]))
                self.assertFalse(result.valid)
                self.assertEqual(result.field, field)

    def test_missing_fields_and_none_request_fail_closed(self):
        result = validate_formation_request(None)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "request")
        request = formation_request()
        del request.formation_goal
        result = validate_formation_request(request)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "formation_goal")
        spec = vehicle_spec(0)
        del spec.goal_heading_free
        result = validate_formation_request(
            formation_request(vehicles=[spec]))
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "vehicles[0].goal_heading_free")

    def test_formation_start_and_goal_must_be_finite(self):
        for field in ("formation_start", "formation_goal"):
            request = formation_request()
            getattr(request, field).theta = math.nan
            with self.subTest(field=field):
                result = validate_formation_request(request)
                self.assertFalse(result.valid)
                self.assertEqual(result.field, field + ".theta")


class FormationTimedTrajectoryContracts(unittest.TestCase):
    def test_valid_common_time_grid_is_accepted(self):
        result = validate_formation_timed_trajectory(make_timed_trajectory())
        self.assertTrue(result.valid, msg=result)
        snapshot = result.snapshot
        self.assertEqual(snapshot.plan_id, 21)
        self.assertEqual(snapshot.formation_id, "wedge_2")
        self.assertEqual(snapshot.vehicle_ids, (0, 1))
        self.assertEqual(snapshot.times_from_start, TRAJECTORY_TIMES)
        self.assertEqual(snapshot.sample_count, 3)
        self.assertEqual(snapshot.vehicle_count, 2)
        status = result.as_status()
        self.assertFalse(status["executable"])
        self.assertFalse(status["independent_audit"])

    def test_empty_and_duplicate_vehicle_specs_fail(self):
        result = validate_formation_timed_trajectory(
            make_timed_trajectory(vehicle_ids=()))
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, INVALID_TIMED_TRAJECTORY)
        self.assertEqual(result.field, "vehicles")

        message = make_timed_trajectory()
        message.vehicles.append(deepcopy(message.vehicles[0]))
        result = validate_formation_timed_trajectory(message)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "vehicles[2].vehicle_id")
        self.assertEqual(result.vehicle_id, 0)

    def test_vehicle_and_trajectory_id_sets_must_match(self):
        missing = make_timed_trajectory(trajectory_ids=(0, 2))
        result = validate_formation_timed_trajectory(missing)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "trajectories")
        self.assertIn("missing=[1]", result.detail)
        self.assertIn("extra=[2]", result.detail)

        duplicate = make_timed_trajectory(trajectory_ids=(0, 0))
        result = validate_formation_timed_trajectory(duplicate)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "trajectories[1].vehicle_id")
        self.assertEqual(result.vehicle_id, 0)

    def test_different_path_lengths_fail(self):
        result = validate_formation_timed_trajectory(
            make_timed_trajectory(points_lengths=(3, 2)))
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "trajectories[1].points")
        self.assertEqual(result.vehicle_id, 1)
        self.assertIn("common time grid", result.detail)

        result = validate_formation_timed_trajectory(
            make_timed_trajectory(trajectory_ids=(0,)))
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "trajectories")

    def test_time_grid_must_be_nonnegative_and_strictly_monotonic(self):
        nonmonotonic = make_timed_trajectory(
            times=((10, 0), (11, 0), (10, 500000000)))
        result = validate_formation_timed_trajectory(nonmonotonic)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "times_from_start[2]")
        self.assertIn("strictly increasing", result.detail)

        equal = make_timed_trajectory(
            times=((10, 0), (11, 0), (11, 0)))
        result = validate_formation_timed_trajectory(equal)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "times_from_start[2]")

        negative = make_timed_trajectory(
            times=((-1, 0), (1, 0), (2, 0)))
        result = validate_formation_timed_trajectory(negative)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "times_from_start[0]")

    def test_every_point_timestamp_must_equal_the_common_grid(self):
        message = make_timed_trajectory(
            point_time_overrides={(0, 2): (11, 0)})
        result = validate_formation_timed_trajectory(message)
        self.assertFalse(result.valid)
        self.assertEqual(result.field,
                         "trajectories[0].points[2].time_from_start")
        self.assertEqual(result.vehicle_id, 0)
        self.assertIn("common grid", result.detail)

    def test_formation_pose_grid_must_match_the_common_times(self):
        result = validate_formation_timed_trajectory(
            make_timed_trajectory(formation_pose_count=2))
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "formation_poses")
        self.assertIn("does not match", result.detail)

    def test_point_modes_and_numeric_fields_are_strict(self):
        for mode, field in ((3, "trajectories[0].points[0].mode"),
                            (-1, "trajectories[0].points[0].mode")):
            message = make_timed_trajectory()
            message.trajectories[0].points[0].mode = mode
            with self.subTest(mode=mode):
                result = validate_formation_timed_trajectory(message)
                self.assertFalse(result.valid)
                self.assertEqual(result.field, field)

        for attribute, field in (
                ("center_speed_mps", "trajectories[0].points[0].center_speed_mps"),
                ("center_yaw_rate_radps",
                 "trajectories[0].points[0].center_yaw_rate_radps"),
                ("mode_control", "trajectories[0].points[0].mode_control")):
            message = make_timed_trajectory()
            setattr(message.trajectories[0].points[0], attribute, math.nan)
            with self.subTest(attribute=attribute):
                result = validate_formation_timed_trajectory(message)
                self.assertFalse(result.valid)
                self.assertEqual(result.field, field)
                self.assertEqual(result.vehicle_id, 0)

        message = make_timed_trajectory()
        message.formation_poses[0].theta = math.inf
        result = validate_formation_timed_trajectory(message)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "formation_poses[0].theta")

    def test_header_and_empty_time_grid_fail(self):
        for kwargs, field in (
                ({"frame_id": "odom"}, "header.frame_id"),
                ({"stamp_secs": 0, "stamp_nsecs": 0}, "header.stamp"),
                ({"times": ()}, "times_from_start")):
            with self.subTest(kwargs=kwargs):
                result = validate_formation_timed_trajectory(
                    make_timed_trajectory(**kwargs))
                self.assertFalse(result.valid)
                self.assertEqual(result.field, field)

        result = validate_formation_timed_trajectory(None)
        self.assertFalse(result.valid)
        self.assertEqual(result.field, "trajectory")


class MessageFreezeContracts(unittest.TestCase):
    def test_existing_single_vehicle_message_bytes_are_untouched(self):
        for filename, expected in SINGLE_VEHICLE_MSG_SHA256.items():
            with self.subTest(filename=filename):
                digest = hashlib.sha256(
                    (MSG_DIR / filename).read_bytes()).hexdigest()
                self.assertEqual(digest, expected)

    def test_new_message_files_and_cmake_inventory(self):
        cmake = (REPO_ROOT / "catkin_ws/src/fwis_v3_msgs/CMakeLists.txt").read_text()
        for filename in NEW_MSG_FILES:
            with self.subTest(filename=filename):
                self.assertTrue((MSG_DIR / filename).is_file())
                self.assertIn(filename, cmake)
        for filename in SINGLE_VEHICLE_MSG_SHA256:
            self.assertIn(filename, cmake)

    def test_new_message_files_freeze_key_field_names(self):
        expectations = {
            "FormationVehicleSpec.msg": (
                "uint32 vehicle_id", "geometry_msgs/Point offset_in_formation",
                "geometry_msgs/Pose2D start_center",
                "geometry_msgs/Pose2D goal_center",
                "bool goal_heading_free", "float64 body_length_m",
                "float64 body_width_m"),
            "FormationPlanRequest.msg": (
                "std_msgs/Header header", "uint32 plan_id",
                "string formation_id", "geometry_msgs/Pose2D formation_start",
                "geometry_msgs/Pose2D formation_goal",
                "fwis_v3_msgs/FormationVehicleSpec[] vehicles",
                "bool allow_reverse"),
            "FormationTimedTrajectory.msg": (
                "geometry_msgs/Pose2D[] formation_poses",
                "duration[] times_from_start",
                "fwis_v3_msgs/FormationVehicleTrajectory[] trajectories"),
        }
        for filename, lines in expectations.items():
            text = (MSG_DIR / filename).read_text()
            for line in lines:
                with self.subTest(filename=filename, line=line):
                    self.assertIn(line, text)


@unittest.skipIf(MESSAGE_MODULE is None,
                 "fwis_v3_msgs is not built; run catkin_make first")
class GeneratedMessageContracts(unittest.TestCase):
    def _assert_roundtrip(self, message):
        from io import BytesIO
        buffer = BytesIO()
        message.serialize(buffer)
        payload = buffer.getvalue()
        decoded = type(message)()
        decoded.deserialize(payload)
        return decoded

    def test_constant_values_are_frozen(self):
        primitive = MESSAGE_MODULE.FormationPrimitive
        self.assertEqual(primitive.TRANSLATION, 0)
        self.assertEqual(primitive.ROTATION, 1)
        self.assertEqual(primitive.COMPOSITE, 2)
        timed_point = MESSAGE_MODULE.FormationTimedPoint
        self.assertEqual(timed_point.ACKERMANN, 0)
        self.assertEqual(timed_point.OBLIQUE, 1)
        self.assertEqual(timed_point.SPIN, 2)

    def test_generated_slot_types_are_frozen(self):
        expectations = {
            "FormationVehicleSpec": [
                "uint32", "geometry_msgs/Point", "geometry_msgs/Pose2D",
                "geometry_msgs/Pose2D", "bool", "float64", "float64"],
            "FormationPlanRequest": [
                "std_msgs/Header", "uint32", "string",
                "geometry_msgs/Pose2D", "geometry_msgs/Pose2D",
                "fwis_v3_msgs/FormationVehicleSpec[]", "bool"],
            "FormationPrimitive": [
                "uint8", "float64", "float64"],
            "FormationPlanSeed": [
                "std_msgs/Header", "uint32", "string",
                "geometry_msgs/Pose2D", "geometry_msgs/Pose2D",
                "fwis_v3_msgs/FormationPrimitive[]"],
            "FormationTimedPoint": [
                "duration", "geometry_msgs/Pose2D", "float64", "float64",
                "uint8", "float64"],
            "FormationVehicleTrajectory": [
                "uint32", "fwis_v3_msgs/FormationTimedPoint[]"],
            "FormationTimedTrajectory": [
                "std_msgs/Header", "uint32", "string",
                "fwis_v3_msgs/FormationVehicleSpec[]",
                "geometry_msgs/Pose2D[]", "duration[]",
                "fwis_v3_msgs/FormationVehicleTrajectory[]"],
        }
        for name, slot_types in expectations.items():
            with self.subTest(message=name):
                self.assertEqual(getattr(MESSAGE_MODULE, name)._slot_types,
                                 slot_types)

    def test_generated_cpp_headers_exist_after_build(self):
        if not DEVEL_INCLUDE.is_dir():
            self.skipTest("catkin devel include directory is absent")
        for name in NEW_MSG_TYPES:
            with self.subTest(message=name):
                self.assertTrue((DEVEL_INCLUDE / (name + ".h")).is_file())

    def test_request_roundtrip_and_validator_accept_real_ros_message(self):
        from genpy import Time
        from geometry_msgs.msg import Point, Pose2D
        from std_msgs.msg import Header
        request_module = MESSAGE_MODULE.FormationPlanRequest
        spec_module = MESSAGE_MODULE.FormationVehicleSpec

        specs = []
        for index in range(len(OFFSETS)):
            start = rigid_center(QF_START, OFFSETS[index])
            goal = rigid_center(QF_GOAL, OFFSETS[index])
            specs.append(spec_module(
                vehicle_id=index,
                offset_in_formation=Point(x=OFFSETS[index][0],
                                          y=OFFSETS[index][1], z=0.0),
                start_center=Pose2D(x=start[0], y=start[1],
                                    theta=START_HEADINGS[index]),
                goal_center=Pose2D(x=goal[0], y=goal[1],
                                   theta=GOAL_HEADINGS[index]),
                goal_heading_free=FREE[index],
                body_length_m=BODY_LENGTH,
                body_width_m=BODY_WIDTH))
        message = request_module(
            header=Header(stamp=Time(secs=100, nsecs=5), frame_id="map"),
            plan_id=17,
            formation_id="wedge_3",
            formation_start=Pose2D(x=QF_START[0], y=QF_START[1],
                                   theta=QF_START[2]),
            formation_goal=Pose2D(x=QF_GOAL[0], y=QF_GOAL[1],
                                  theta=QF_GOAL[2]),
            vehicles=specs,
            allow_reverse=False)

        decoded = self._assert_roundtrip(message)
        self.assertEqual(decoded.plan_id, 17)
        self.assertEqual([vehicle.vehicle_id for vehicle in decoded.vehicles],
                         [0, 1, 2])
        result = validate_formation_request(decoded)
        self.assertTrue(result.valid, msg=result)
        self.assertEqual(result.snapshot.model.vehicle_ids, (0, 1, 2))

    def test_timed_trajectory_roundtrip_and_validator_accept_ros_message(self):
        from genpy import Duration, Time
        from geometry_msgs.msg import Point, Pose2D
        from std_msgs.msg import Header

        specs = []
        for index in (0, 1):
            start = rigid_center(QF_START, OFFSETS[index])
            goal = rigid_center(QF_GOAL, OFFSETS[index])
            specs.append(MESSAGE_MODULE.FormationVehicleSpec(
                vehicle_id=index,
                offset_in_formation=Point(
                    x=OFFSETS[index][0], y=OFFSETS[index][1], z=0.0),
                start_center=Pose2D(x=start[0], y=start[1],
                                    theta=START_HEADINGS[index]),
                goal_center=Pose2D(x=goal[0], y=goal[1],
                                   theta=GOAL_HEADINGS[index]),
                goal_heading_free=FREE[index],
                body_length_m=BODY_LENGTH,
                body_width_m=BODY_WIDTH))
        points = []
        for vehicle_id in (0, 1):
            points.append(MESSAGE_MODULE.FormationVehicleTrajectory(
                vehicle_id=vehicle_id,
                points=[
                    MESSAGE_MODULE.FormationTimedPoint(
                        time_from_start=Duration(secs=secs, nsecs=nsecs),
                        center_pose=Pose2D(x=4.0, y=4.0, theta=0.1 * vehicle_id),
                        center_speed_mps=0.5,
                        center_yaw_rate_radps=0.0,
                        mode=0,
                        mode_control=0.0)
                    for secs, nsecs in TRAJECTORY_TIMES]))
        message = MESSAGE_MODULE.FormationTimedTrajectory(
            header=Header(stamp=Time(secs=100, nsecs=5), frame_id="map"),
            plan_id=21,
            formation_id="wedge_2",
            vehicles=specs,
            formation_poses=[Pose2D(x=4.0, y=4.0, theta=0.2)
                             for _ in TRAJECTORY_TIMES],
            times_from_start=[Duration(secs=secs, nsecs=nsecs)
                              for secs, nsecs in TRAJECTORY_TIMES],
            trajectories=points)

        decoded = self._assert_roundtrip(message)
        self.assertEqual(len(decoded.trajectories), 2)
        self.assertEqual(len(decoded.trajectories[0].points), 3)
        result = validate_formation_timed_trajectory(decoded)
        self.assertTrue(result.valid, msg=result)
        self.assertEqual(result.snapshot.times_from_start, TRAJECTORY_TIMES)

    def test_seed_and_primitive_roundtrip(self):
        from genpy import Time
        from geometry_msgs.msg import Pose2D
        from std_msgs.msg import Header

        message = MESSAGE_MODULE.FormationPlanSeed(
            header=Header(stamp=Time(secs=100, nsecs=5), frame_id="map"),
            plan_id=17,
            formation_id="wedge_3",
            formation_start=Pose2D(x=2.0, y=2.0, theta=0.35),
            formation_goal=Pose2D(x=6.0, y=3.0, theta=-0.4),
            primitives=[MESSAGE_MODULE.FormationPrimitive(
                geometry_type=MESSAGE_MODULE.FormationPrimitive.TRANSLATION,
                amount=2.0, control=0.0)])
        decoded = self._assert_roundtrip(message)
        self.assertEqual(decoded.primitives[0].geometry_type,
                         MESSAGE_MODULE.FormationPrimitive.TRANSLATION)
        self.assertEqual(decoded.plan_id, 17)

    def test_single_vehicle_messages_still_import_unchanged(self):
        for name in ("ModePrimitive", "PlanSeed", "PlanSeedCandidates",
                     "TimedPoint", "TimedTrajectory"):
            with self.subTest(message=name):
                self.assertTrue(hasattr(MESSAGE_MODULE, name))
        self.assertEqual(MESSAGE_MODULE.PlanSeed._slot_types, [
            "std_msgs/Header", "uint32", "geometry_msgs/Pose2D",
            "geometry_msgs/Pose2D", "fwis_v3_msgs/ModePrimitive[]"])
        self.assertEqual(MESSAGE_MODULE.TimedTrajectory._slot_types, [
            "std_msgs/Header", "uint32", "fwis_v3_msgs/TimedPoint[]"])
        self.assertEqual(MESSAGE_MODULE.TimedPoint._slot_types, [
            "duration", "geometry_msgs/Pose2D", "float64", "uint8"])


if __name__ == "__main__":
    unittest.main()
