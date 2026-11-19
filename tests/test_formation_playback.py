"""S10 ROS-independent formation playback tests.

These tests never start ROS.  They pin the common-clock interpolation, marker
namespace/id rules, GEOMETRY_ONLY labelling and fail-closed conversion used by
``fwis_v3_playback/scripts/formation_playback.py``.
"""
from pathlib import Path
import json
import math
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    ACKERMANN,
    CLEARED,
    GEOMETRY_ONLY_LABEL,
    HOLD_END,
    HOLD_START,
    OBLIQUE,
    PLAYING,
    PlaybackMarkerOptions,
    PlaybackValidationError,
    SPIN,
    build_markers,
    clear_marker_specs,
    frame_at,
    normalize_formation_id,
    status_document,
    terminal_errors,
    tf_frame_prefix,
    trajectory_from_fields,
    vehicle_namespace,
    vehicle_path_samples,
)


def point(time, x, y, theta, mode=ACKERMANN, control=0.0, speed=1.0):
    return (time, x, y, theta, speed, 0.0, mode, control)


def fields(formation_id="wedge_3", plan_id=7, times=(0.0, 1.0, 2.0),
           samples=None, poses=None, stamp=(100, 500000000)):
    if samples is None:
        samples = {
            0: (point(0.0, 0.0, 0.0, 0.0),
                point(1.0, 1.0, 0.0, 0.0),
                point(2.0, 2.0, 0.0, 0.0)),
            1: (point(0.0, 0.0, 1.2, math.pi / 2.0, OBLIQUE,
                      math.pi / 4.0, 0.8),
                point(1.0, 1.0, 1.2, math.pi / 2.0, OBLIQUE,
                      math.pi / 4.0, 0.8),
                point(2.0, 2.0, 1.2, math.pi / 2.0, OBLIQUE,
                      math.pi / 4.0, 0.8)),
        }
    if poses is None:
        poses = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    return {
        "frame_id": "map",
        "stamp": stamp,
        "plan_id": plan_id,
        "formation_id": formation_id,
        "vehicles": (
            {"vehicle_id": 0, "offset": (0.0, 0.0),
             "body_length_m": 0.738, "body_width_m": 0.5},
            {"vehicle_id": 1, "offset": (0.0, 1.2),
             "body_length_m": 0.738, "body_width_m": 0.5},
        ),
        "formation_poses": poses,
        "times_from_start": times,
        "trajectories": (
            {"vehicle_id": 0, "points": samples[0]},
            {"vehicle_id": 1, "points": samples[1]},
        ),
    }


def trajectory(**values):
    return trajectory_from_fields(fields(**values))


def execute_status(plan_id=7):
    return {
        "schema": "formation_optimizer_status/v1",
        "node_state": "FEASIBLE_PUBLISHED",
        "plan_id": plan_id,
        "formation_id": "wedge_3",
        "status": "FEASIBLE",
        "feasible": True,
        "executable": True,
        "trajectory_published": True,
        "reason": "OK",
        "pipeline": {
            "selection": {
                "selected_candidate_id": 1,
                "evaluations": [{
                    "status": "FEASIBLE",
                    "plan_id": plan_id,
                    "metrics": {
                        "duration_s": 2.0,
                        "max_rigidity_error_m": 1e-15,
                        "minimum_obstacle_clearance_m": 0.8,
                        "minimum_intervehicle_clearance_m": 0.32,
                        "dynamic_peak_ratio": 0.91,
                        "mode_switch_count": 2,
                    },
                }],
            },
        },
    }


class PlaybackTrajectoryTest(unittest.TestCase):

    def test_fields_validate_and_report_shared_samples(self):
        model = trajectory()
        self.assertEqual(model.plan_id, 7)
        self.assertEqual(model.formation_id, "wedge_3")
        self.assertEqual(model.vehicle_ids, (0, 1))
        self.assertEqual(model.sample_count, 3)
        self.assertAlmostEqual(model.duration, 2.0)
        self.assertAlmostEqual(model.stamp_seconds, 100.5)

    def test_zero_stamp_is_not_a_common_clock(self):
        with self.assertRaises(PlaybackValidationError):
            trajectory(stamp=(0, 0))

    def test_time_axis_must_start_at_zero_and_increase(self):
        with self.assertRaises(PlaybackValidationError):
            trajectory(times=(0.1, 1.0, 2.0))
        with self.assertRaises(PlaybackValidationError):
            trajectory(times=(0.0, 1.0, 1.0))

    def test_per_vehicle_time_must_match_the_shared_axis(self):
        bad = list(fields()["trajectories"][0]["points"])
        bad[1] = point(1.1, 1.0, 0.0, 0.0)
        samples = {
            0: tuple(bad),
            1: fields()["trajectories"][1]["points"],
        }
        with self.assertRaises(PlaybackValidationError):
            trajectory(samples=samples)

    def test_duplicate_vehicle_ids_fail_closed(self):
        values = fields()
        values["vehicles"] = (values["vehicles"][0], values["vehicles"][0])
        with self.assertRaises(PlaybackValidationError):
            trajectory_from_fields(values)

    def test_nonfinite_sample_fails_closed(self):
        samples = {
            0: (point(0.0, 0.0, 0.0, 0.0),
                point(1.0, float("nan"), 0.0, 0.0),
                point(2.0, 2.0, 0.0, 0.0)),
            1: fields()["trajectories"][1]["points"],
        }
        with self.assertRaises(PlaybackValidationError):
            trajectory(samples=samples)


class PlaybackClockTest(unittest.TestCase):

    def test_all_vehicles_use_one_clock_and_one_fraction(self):
        model = trajectory()
        frame = frame_at(model, 0.25)
        self.assertEqual(frame.state, PLAYING)
        self.assertEqual(frame.lower_index, 0)
        self.assertEqual(frame.upper_index, 1)
        self.assertAlmostEqual(frame.fraction, 0.25)
        self.assertAlmostEqual(frame.formation_x, 0.25)
        states = frame.vehicle_map()
        self.assertAlmostEqual(states[0].center_x, 0.25)
        self.assertAlmostEqual(states[1].center_x, 0.25)
        self.assertAlmostEqual(states[1].center_y, 1.2)

    def test_before_stamp_holds_the_first_sample(self):
        frame = frame_at(trajectory(), -3.0)
        self.assertEqual(frame.state, HOLD_START)
        self.assertEqual(frame.clamped_elapsed_s, 0.0)
        self.assertAlmostEqual(frame.vehicles[1].center_x, 0.0)

    def test_after_end_holds_the_final_sample(self):
        model = trajectory()
        frame = frame_at(model, 12.0)
        self.assertEqual(frame.state, HOLD_END)
        self.assertTrue(frame.terminal)
        self.assertAlmostEqual(frame.clamped_elapsed_s, 2.0)
        self.assertAlmostEqual(frame.formation_x, 2.0)
        center_error, heading_error = terminal_errors(model, frame)
        self.assertLessEqual(center_error, 1e-12)
        self.assertLessEqual(heading_error, 1e-12)

    def test_heading_interpolation_takes_the_short_arc(self):
        samples = {
            0: (point(0.0, 0.0, 0.0, 3.0),
                point(1.0, 0.0, 0.0, -3.0),
                point(2.0, 0.0, 0.0, -3.0)),
            1: fields()["trajectories"][1]["points"],
        }
        model = trajectory(samples=samples)
        frame = frame_at(model, 0.5)
        value = frame.vehicles[0].theta
        self.assertLessEqual(abs(abs(value) - math.pi), 1e-6)

    def test_mode_is_piecewise_constant_from_the_lower_sample(self):
        samples = {
            0: (point(0.0, 0.0, 0.0, 0.0, ACKERMANN),
                point(1.0, 1.0, 0.0, 0.0, SPIN),
                point(2.0, 2.0, 0.0, 0.0, SPIN)),
            1: fields()["trajectories"][1]["points"],
        }
        model = trajectory(samples=samples)
        self.assertEqual(frame_at(model, 0.999).vehicles[0].mode, ACKERMANN)
        self.assertEqual(frame_at(model, 1.001).vehicles[0].mode, SPIN)

    def test_single_sample_hold_is_the_static_terminal_state(self):
        model = trajectory(
            times=(0.0,),
            samples={
                0: (point(0.0, 0.0, 0.0, 0.0, speed=0.0),),
                1: (point(0.0, 0.0, 1.2, math.pi / 2.0, OBLIQUE,
                          0.0, 0.0),),
            },
            poses=((0.0, 0.0, 0.0),))
        self.assertEqual(model.duration, 0.0)
        frame = frame_at(model, 1.0)
        self.assertEqual(frame.state, HOLD_END)
        self.assertTrue(frame.terminal)
        self.assertEqual(frame.lower_index, 0)
        self.assertEqual(frame.upper_index, 0)
        self.assertEqual(frame.fraction, 0.0)
        vehicle = frame.vehicle_map()[1]
        self.assertAlmostEqual(vehicle.center_x, 0.0)
        self.assertAlmostEqual(vehicle.center_y, 1.2)
        self.assertAlmostEqual(vehicle.theta, math.pi / 2.0)
        self.assertEqual(vehicle.center_speed_mps, 0.0)
        document = status_document(
            model, frame,
            {"schema": "formation_optimizer_status/v1", "plan_id": 7,
             "status": "NO_MOTION", "feasible": True, "executable": True,
             "trajectory_published": True, "reason": "NO_MOTION"})
        self.assertTrue(document["executable"])
        self.assertEqual(document["certification"], "NO_MOTION")
        self.assertEqual(document["duration_s"], 0.0)
        self.assertTrue(document["terminal"])
        json.dumps(document, sort_keys=True, allow_nan=False)

    def test_status_document_names_the_single_time_source(self):
        model = trajectory()
        frame = frame_at(model, 0.5)
        document = status_document(
            model, frame, execute_status(), marker_count=3,
            namespaces=("formation/wedge_3/vehicle_0",))
        self.assertEqual(document["schema"], "formation_playback/v1")
        self.assertEqual(document["time_source"],
                         "header_stamp_common_clock")
        self.assertEqual(document["state"], PLAYING)
        self.assertEqual(document["vehicle_ids"], [0, 1])
        self.assertEqual(document["trajectory_stamp"],
                         {"secs": 100, "nsecs": 500000000})
        json.dumps(document, sort_keys=True, allow_nan=False)


class PlaybackMarkerTest(unittest.TestCase):

    def test_vehicle_namespace_contains_formation_and_vehicle(self):
        namespace = vehicle_namespace("wedge_3", 1)
        self.assertEqual(namespace, "formation/wedge_3/vehicle_1")
        self.assertIn("wedge_3", namespace)
        self.assertIn("vehicle_1", namespace)

    def test_tf_frames_are_unique_and_formation_scoped(self):
        first = tf_frame_prefix("wedge_3")
        second = tf_frame_prefix("wedge 3")
        self.assertEqual(first, "formation_wedge_3")
        self.assertNotEqual(first, second)
        self.assertNotEqual(tf_frame_prefix("wedge/3"),
                            tf_frame_prefix("wedge_3"))
        self.assertTrue(normalize_formation_id("a/b c").startswith("a_b_c_"))

    def test_marker_ids_are_unique_and_every_vehicle_is_present(self):
        model = trajectory()
        frame = frame_at(model, 0.5)
        markers, namespaces, audit, executable = build_markers(
            model, frame)
        ids = [marker.marker_id for marker in markers]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertFalse(executable)
        self.assertEqual(audit, "AUDIT_PENDING")
        status_marker = next(
            marker for marker in markers
            if marker.namespace == "formation/wedge_3/status")
        self.assertIn(GEOMETRY_ONLY_LABEL, status_marker.text)
        for vehicle_id in (0, 1):
            namespace = vehicle_namespace("wedge_3", vehicle_id)
            self.assertIn(namespace, namespaces)
            self.assertTrue(any(
                marker.namespace == namespace for marker in markers))
        names = {marker.namespace for marker in markers}
        self.assertIn("formation/wedge_3/reference", names)
        self.assertIn("formation/wedge_3/links", names)
        self.assertIn("formation/wedge_3/status", names)

    def test_obb_uses_the_body_heading_not_the_formation_heading(self):
        model = trajectory()
        frame = frame_at(model, 0.0)
        markers, _, _, _ = build_markers(model, frame)
        namespace = vehicle_namespace("wedge_3", 1)
        body = next(
            marker for marker in markers
            if marker.namespace == namespace
            and marker.marker_type == "LINE_STRIP"
            and len(marker.points) == 5
            and abs(marker.points[0][0] - marker.points[4][0]) < 1e-12
            and abs(marker.points[0][1] - marker.points[4][1]) < 1e-12)
        # Vehicle 1 is at (0, 1.2) with body yaw pi/2: its OBB is long in y.
        ys = [value[1] for value in body.points]
        xs = [value[0] for value in body.points]
        self.assertAlmostEqual(max(ys) - min(ys), 0.738, places=6)
        self.assertAlmostEqual(max(xs) - min(xs), 0.5, places=6)

    def test_vehicle_path_samples_keep_every_authoritative_sample(self):
        samples = {
            0: (point(0.0, 0.0, 0.0, 0.0, ACKERMANN),
                point(1.0, 1.0, 0.0, 0.5, OBLIQUE),
                point(2.0, 2.0, 0.0, 1.0, SPIN)),
            1: fields()["trajectories"][1]["points"],
        }
        model = trajectory(samples=samples)
        paths = vehicle_path_samples(model)
        self.assertEqual(set(paths), {0, 1})
        self.assertEqual(
            paths[0],
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.5), (2.0, 0.0, 1.0)))
        self.assertEqual(
            paths[1],
            tuple((item[1], item[2], item[3])
                  for item in samples[1]))
        frame = frame_at(model, 1.0)
        markers, _, _, _ = build_markers(model, frame)
        # The marker array must not carry a second, non-adjustable copy of
        # the full path; RViz Path displays own the full-centre polylines.
        namespace = vehicle_namespace("wedge_3", 0)
        vehicle_markers = [
            marker for marker in markers if marker.namespace == namespace]
        self.assertEqual(len(vehicle_markers), 4)
        self.assertFalse(any(marker.colors for marker in vehicle_markers))

    def test_audited_status_allows_feasible_label(self):
        model = trajectory()
        frame = frame_at(model, 0.5)
        markers, _, audit, executable = build_markers(
            model, frame, status=execute_status())
        self.assertTrue(executable)
        self.assertEqual(audit, "FEASIBLE")
        status = next(
            marker for marker in markers
            if marker.namespace == "formation/wedge_3/status")
        self.assertIn("FEASIBLE", status.text)
        self.assertNotIn(GEOMETRY_ONLY_LABEL, status.text)
        self.assertIn("T_end=2.000", status.text)

    def test_unpublished_status_keeps_geometry_only_label(self):
        model = trajectory()
        frame = frame_at(model, 0.5)
        status = execute_status()
        status["trajectory_published"] = False
        status["executable"] = False
        markers, _, _, executable = build_markers(
            model, frame, status=status)
        self.assertFalse(executable)
        label = next(
            marker for marker in markers
            if marker.namespace == "formation/wedge_3/status")
        self.assertIn(GEOMETRY_ONLY_LABEL, label.text)

    def test_clear_is_a_single_deleteall_marker(self):
        specs = clear_marker_specs("map")
        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0].marker_type, "DELETEALL")
        self.assertEqual(specs[0].action, "DELETEALL")

    def test_cleared_and_invalid_status_documents_are_strict(self):
        cleared = status_document(cleared=True)
        self.assertEqual(cleared["state"], CLEARED)
        self.assertTrue(cleared["cleared"])
        self.assertEqual(cleared["marker_count"], 0)
        invalid = status_document(invalid_reason="bad stamp")
        self.assertEqual(invalid["state"], "INVALID")
        self.assertTrue(invalid["geometry_only"])
        for document in (cleared, invalid):
            json.dumps(document, sort_keys=True, allow_nan=False)


if __name__ == "__main__":
    unittest.main()