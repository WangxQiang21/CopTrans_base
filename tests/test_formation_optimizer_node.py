"""S9 ROS-independent node pipeline and transactional publication tests.

These tests do not start ROS.  They exercise the same ``FormationNodeCore``
used by the ROS adapter, the S6->S7->S8->S9 pipeline, the plain trajectory
field conversion, and the exact map conversion helper.
"""
from pathlib import Path
import json
import math
import sys
from types import SimpleNamespace
import unittest

import numpy as np
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts"))

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    FormationValidationError,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    GEOMETRY_ONLY_NOT_EXECUTABLE,
    REJECTED,
)
from dataclasses import replace
from fwis_v3_optimizer.formation_request import validate_formation_request
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits

from formation_optimizer_node import (
    FormationNodeCore,
    FormationPipelineConfig,
    FormationPlanningPipeline,
    grid_from_occupancy_grid,
    timed_trajectory_fields,
)
from fwis_v3_optimizer.formation_quality import FormationQualityConfig


def primitive(geometry_type, dx=0.0, dy=0.0, dpsi=0.0,
              pivot_x=0.0, pivot_y=0.0):
    return SimpleNamespace(
        geometry_type=geometry_type, dx=dx, dy=dy, dpsi=dpsi,
        pivot_x=pivot_x, pivot_y=pivot_y)


def header(secs=123, nsecs=456):
    return SimpleNamespace(
        frame_id="map", stamp=SimpleNamespace(secs=secs, nsecs=nsecs))


def pose(x, y, theta=0.0):
    return SimpleNamespace(x=x, y=y, theta=theta)


def vehicle_spec(vehicle_id, offset_x, offset_y, start, goal,
                 goal_heading_free=False):
    return SimpleNamespace(
        vehicle_id=vehicle_id,
        offset_in_formation=SimpleNamespace(
            x=offset_x, y=offset_y, z=0.0),
        start_center=pose(*start),
        goal_center=pose(*goal),
        goal_heading_free=goal_heading_free,
        body_length_m=0.738,
        body_width_m=0.500)


def request_message(plan_id=41, formation_id="s9_node",
                    start=(0.0, 0.0, 0.0), goal=(2.0, 0.0, 0.0),
                    dpsi=0.0, allow_reverse=False):
    offsets = (VehicleOffset(0, 0.0, 0.0), VehicleOffset(1, 1.2, 0.0))
    model = FormationModel(offsets)
    start_centers = model.centers(FormationPose(*start))
    goal_centers = model.centers(FormationPose(*goal))
    start_headings = (0.0, math.pi / 2.0) if dpsi else (0.0, 0.0)
    goal_headings = ((dpsi, math.pi / 2.0 + dpsi) if dpsi
                     else (0.0, 0.0))
    vehicles = []
    for index, offset in enumerate(offsets):
        vehicles.append(vehicle_spec(
            offset.vehicle_id, offset.x, offset.y,
            (start_centers[offset.vehicle_id][0],
             start_centers[offset.vehicle_id][1],
             start_headings[index]),
            (goal_centers[offset.vehicle_id][0],
             goal_centers[offset.vehicle_id][1],
             goal_headings[index])))
    return SimpleNamespace(
        header=header(), plan_id=plan_id, formation_id=formation_id,
        formation_start=pose(*start), formation_goal=pose(*goal),
        vehicles=vehicles, allow_reverse=allow_reverse)


def seed_message(request, primitives):
    return SimpleNamespace(
        header=header(), plan_id=request.plan_id,
        formation_id=request.formation_id,
        formation_start=request.formation_start,
        formation_goal=request.formation_goal,
        primitives=tuple(primitives))


def empty_grid():
    return Grid(np.zeros((120, 160), dtype=bool), 0.05, -2.0, -1.5)


def pipeline_config(**quality_overrides):
    quality_values = {
        "collision_margin_m": 0.05,
        "sweep_spacing_m": 0.01,
        "publication_dt_s": 0.02,
    }
    quality_values.update(quality_overrides)
    return FormationPipelineConfig(
        quality_config=FormationQualityConfig(**quality_values),
        enable_nlp=False,
        max_candidates=2,
        max_schedule_combinations=8,
        max_schedules_per_block=2)


def straight_request(plan_id=41):
    return request_message(plan_id=plan_id)


def rotation_request(plan_id=42):
    dpsi = 0.5
    return request_message(
        plan_id=plan_id, formation_id="s9_rotation",
        goal=(0.0, 0.0, dpsi), dpsi=dpsi)


def publishable_core(config=None):
    core = FormationNodeCore(pipeline_config=config or pipeline_config())
    core.on_map(empty_grid(), None, "map", stamp=(1000, 0))
    return core


def publish(core, request, seed_primitive, seed_stamp=(123, 456)):
    request_event = core.on_request(
        request, publication_stamp=(1001, 0))
    seed = seed_message(request, (seed_primitive,))
    seed.header.stamp = SimpleNamespace(
        secs=seed_stamp[0], nsecs=seed_stamp[1])
    seed_event = core.on_seed(seed, publication_stamp=(1002, 0))
    return request_event, seed, seed_event


class FormationNodePipelineTest(unittest.TestCase):

    def test_straight_pipeline_feasible_and_json_safe(self):
        request = straight_request()
        snapshot = validate_formation_request(request).snapshot
        seed = seed_message(request, (primitive(0, dx=2.0),))
        result = FormationPlanningPipeline(pipeline_config()).run(
            snapshot, seed, empty_grid())
        self.assertEqual(result.status, FEASIBLE, result.detail)
        self.assertIsNotNone(result.selection)
        self.assertEqual(result.selection.status, FEASIBLE)
        self.assertGreaterEqual(len(result.candidates), 1)
        self.assertEqual(result.candidates[0].candidate_id, 1)
        self.assertEqual(result.candidates[0].origin, "s6_seed")
        self.assertAlmostEqual(
            result.selection.selected_timed.duration,
            result.selected_timed.duration)
        status = result.as_status()
        self.assertEqual(status["status"], FEASIBLE)
        self.assertTrue(status["executable"])
        self.assertFalse(status["trajectory_published"])
        self.assertTrue(status["collision_certificate"])
        self.assertTrue(status["selection"]["evaluations"][0]["feasible"])
        # The adapter serializes this exact structure; it must be strict JSON.
        json.dumps(status, sort_keys=True, allow_nan=False)

    def test_rotation_pipeline_fields_keep_formation_and_body_headings(self):
        request = rotation_request()
        snapshot = validate_formation_request(request).snapshot
        seed = seed_message(request, (primitive(1, dpsi=0.5),))
        result = FormationPlanningPipeline(pipeline_config()).run(
            snapshot, seed, empty_grid())
        self.assertEqual(result.status, FEASIBLE, result.detail)
        timed = result.selected_timed
        fields = timed_trajectory_fields(
            snapshot, timed, (2000, 0))
        self.assertEqual(fields["plan_id"], 42)
        self.assertEqual(fields["stamp"], (2000, 0))
        self.assertEqual(len(fields["formation_poses"]),
                         len(fields["times_from_start"]))
        self.assertAlmostEqual(fields["formation_poses"][0][2], 0.0)
        self.assertAlmostEqual(fields["formation_poses"][-1][2], 0.5)
        vehicle = {item["vehicle_id"]: item
                   for item in fields["trajectories"]}[1]
        points = vehicle["points"]
        self.assertAlmostEqual(points[0][3], math.pi / 2.0)
        self.assertAlmostEqual(points[-1][3], math.pi / 2.0 + 0.5)
        # Body heading is not silently replaced by formation heading.
        self.assertNotAlmostEqual(points[0][3],
                                  fields["formation_poses"][0][2])
        self.assertNotAlmostEqual(points[-1][3],
                                  fields["formation_poses"][-1][2])
        for first, second in zip(fields["times_from_start"],
                                 fields["times_from_start"][1:]):
            self.assertGreater(second, first)
        with self.assertRaises(FormationValidationError):
            timed_trajectory_fields(snapshot, timed, (0, 0))

    def test_retiming_failure_is_geometry_only_not_feasible(self):
        config = pipeline_config(timing_limits=FormationTimingLimits(
            v_max=0.05, max_block_duration_s=5.0))
        request = straight_request()
        snapshot = validate_formation_request(request).snapshot
        seed = seed_message(request, (primitive(0, dx=2.0),))
        result = FormationPlanningPipeline(config).run(
            snapshot, seed, empty_grid())
        self.assertEqual(result.status, GEOMETRY_ONLY_NOT_EXECUTABLE)
        self.assertIsNone(result.selection)
        self.assertFalse(result.candidates)
        self.assertTrue(result.rejections)
        self.assertTrue(all(
            item.stage == "retiming" and
            item.failure_code == "FORMATION_RETIMING_INFEASIBLE"
            for item in result.rejections))
        status = result.as_status()
        self.assertFalse(status["executable"])
        self.assertFalse(status["trajectory_published"])
        self.assertEqual(status["status"], GEOMETRY_ONLY_NOT_EXECUTABLE)
        json.dumps(status, sort_keys=True, allow_nan=False)

    def test_obstacle_is_rejected_before_feasible(self):
        request = straight_request()
        snapshot = validate_formation_request(request).snapshot
        seed = seed_message(request, (primitive(0, dx=2.0),))
        grid = empty_grid()
        grid.occ[30, 60] = True  # x=1.0, y=0.0: mid-sweep obstacle
        result = FormationPlanningPipeline(pipeline_config()).run(
            snapshot, seed, grid)
        self.assertEqual(result.status, REJECTED)
        self.assertIsNone(result.selected_timed)
        self.assertIsNone(result.selection)
        self.assertNotEqual(result.status, FEASIBLE)
        status = result.as_status()
        self.assertFalse(status["executable"])
        self.assertFalse(status["trajectory_published"])
        json.dumps(status, sort_keys=True, allow_nan=False)

    def test_optimizer_exception_is_a_structured_rejection(self):
        config = replace(pipeline_config(), enable_nlp=True)
        request = straight_request()
        snapshot = validate_formation_request(request).snapshot
        seed = seed_message(request, (primitive(0, dx=2.0),))
        with patch(
                "formation_optimizer_node.optimize_continuous_plan",
                side_effect=RuntimeError("solver exploded")):
            result = FormationPlanningPipeline(config).run(
                snapshot, seed, empty_grid())
        self.assertEqual(result.status, REJECTED)
        self.assertIsNone(result.selection)
        self.assertFalse(result.candidates)
        self.assertTrue(result.rejections)
        rejection = result.rejections[0]
        self.assertEqual(rejection.stage, "optimization")
        self.assertEqual(rejection.failure_code, "FORMATION_NLP_INFEASIBLE")
        self.assertIn("solver exploded", rejection.detail)
        self.assertFalse(result.as_status()["executable"])

    def test_invalid_seed_never_reaches_feasible(self):
        request = straight_request()
        snapshot = validate_formation_request(request).snapshot
        seed = seed_message(request, (primitive(9, dx=1.0),))
        result = FormationPlanningPipeline(pipeline_config()).run(
            snapshot, seed, empty_grid())
        self.assertNotEqual(result.status, FEASIBLE)
        self.assertIsNone(result.selection)
        self.assertTrue(result.rejections)
        self.assertTrue(all(item.stage == "build"
                            for item in result.rejections))

    def test_grid_conversion_fails_closed(self):
        valid = SimpleNamespace(
            header=header(),
            info=SimpleNamespace(
                width=4, height=3, resolution=0.5,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=-1.0, y=-0.5, z=0.0),
                    orientation=SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0))),
            data=[0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0])
        grid, error = grid_from_occupancy_grid(valid)
        self.assertIsNone(error)
        self.assertIsNotNone(grid)
        self.assertEqual(grid.occ.shape, (3, 4))
        self.assertTrue(grid.occ[1, 1])  # unknown is conservatively occupied

        wrong_frame = SimpleNamespace(
            header=SimpleNamespace(
                frame_id="odom",
                stamp=SimpleNamespace(secs=1, nsecs=0)),
            info=valid.info, data=valid.data)
        grid, error = grid_from_occupancy_grid(wrong_frame)
        self.assertIsNone(grid)
        self.assertIn("frame_id", error)

        rotated = SimpleNamespace(
            header=header(),
            info=SimpleNamespace(
                width=4, height=3, resolution=0.5,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=SimpleNamespace(
                        x=0.0, y=0.0,
                        z=math.sin(math.pi / 4.0),
                        w=math.cos(math.pi / 4.0)))),
            data=valid.data)
        grid, error = grid_from_occupancy_grid(rotated)
        self.assertIsNone(grid)
        self.assertIn("rotated", error)

        short_data = SimpleNamespace(
            header=header(), info=valid.info, data=[0] * 5)
        grid, error = grid_from_occupancy_grid(short_data)
        self.assertIsNone(grid)
        self.assertIn("data length", error)

        with self.assertRaises(FormationValidationError):
            timed_trajectory_fields(object(), object(), (1, 0))


class FormationNodeTransactionTest(unittest.TestCase):

    def test_request_then_seed_publishes_one_transaction(self):
        core = publishable_core()
        request = rotation_request()
        request_event, seed, seed_event = publish(
            core, request, primitive(1, dpsi=0.5))
        self.assertEqual(request_event.status["node_state"],
                         "REQUEST_VALIDATED")
        self.assertEqual(request_event.status["status"], "VALIDATED")
        self.assertFalse(request_event.status["feasible"])
        self.assertFalse(request_event.status["trajectory_published"])
        self.assertTrue(request_event.clear)

        self.assertEqual(seed_event.status["node_state"],
                         "FEASIBLE_PUBLISHED")
        self.assertEqual(seed_event.status["status"], FEASIBLE)
        self.assertTrue(seed_event.status["feasible"])
        self.assertTrue(seed_event.status["executable"])
        self.assertTrue(seed_event.status["trajectory_published"])
        # The request event already performed the transactional clear.
        self.assertFalse(seed_event.clear)
        self.assertIsNotNone(seed_event.trajectory_fields)
        self.assertEqual(core.gate.published_plan_id, request.plan_id)
        self.assertEqual(seed_event.trajectory_fields["stamp"], (1002, 0))
        quality = seed_event.quality
        self.assertTrue(quality["trajectory_published"])
        self.assertTrue(quality["collision_certificate"])
        self.assertTrue(quality["independent_audit"])
        self.assertEqual(quality["pipeline"]["status"], FEASIBLE)
        self.assertEqual(quality["publication"]["status"], "PUBLISH")
        json.dumps(seed_event.status, sort_keys=True, allow_nan=False)
        json.dumps(seed_event.quality, sort_keys=True, allow_nan=False)

    def test_duplicate_and_stale_seeds_keep_confirmed_publication(self):
        core = publishable_core()
        request = straight_request()
        _, seed, first = publish(core, request, primitive(0, dx=2.0))
        self.assertTrue(first.status["trajectory_published"])
        self.assertEqual(core.gate.published_plan_id, request.plan_id)

        duplicate = core.on_seed(seed, publication_stamp=(1003, 0))
        self.assertEqual(duplicate.status["node_state"], "DUPLICATE_SEED")
        self.assertFalse(duplicate.clear)
        self.assertIsNone(duplicate.trajectory_fields)
        self.assertEqual(core.gate.published_plan_id, request.plan_id)

        stale = seed_message(request, (primitive(0, dx=2.0),))
        stale.plan_id = request.plan_id - 1
        stale.header.stamp = SimpleNamespace(secs=124, nsecs=456)
        event = core.on_seed(stale, publication_stamp=(1004, 0))
        self.assertEqual(event.status["node_state"], "SEED_STALE")
        self.assertFalse(event.clear)
        self.assertEqual(core.gate.published_plan_id, request.plan_id)

        invalid = seed_message(request, (primitive(9, dx=1.0),))
        invalid.header.stamp = SimpleNamespace(secs=125, nsecs=456)
        event = core.on_seed(invalid, publication_stamp=(1005, 0))
        self.assertIn(event.status["node_state"],
                      ("REJECTED", "GEOMETRY_ONLY_NOT_EXECUTABLE"))
        self.assertFalse(event.status["trajectory_published"])
        self.assertTrue(event.clear)
        self.assertIsNone(event.trajectory_fields)
        self.assertIsNone(core.gate.published_plan_id)
        self.assertEqual(event.status["status"], REJECTED)

    def test_new_request_clears_previous_publication(self):
        core = publishable_core()
        request = straight_request(plan_id=41)
        _, _, first = publish(core, request, primitive(0, dx=2.0))
        self.assertTrue(first.status["trajectory_published"])

        second = straight_request(plan_id=42)
        event = core.on_request(second, publication_stamp=(1006, 0))
        self.assertEqual(event.status["node_state"], "REQUEST_VALIDATED")
        self.assertTrue(event.clear)
        self.assertFalse(event.status["trajectory_published"])
        self.assertIsNone(core.gate.published_plan_id)
        self.assertEqual(event.status["plan_id"], 42)

    def test_invalid_request_clears_previous_publication(self):
        core = publishable_core()
        request = straight_request()
        _, _, first = publish(core, request, primitive(0, dx=2.0))
        self.assertTrue(first.status["trajectory_published"])

        bad = SimpleNamespace(
            header=header(), plan_id=43, formation_id="bad",
            formation_start=pose(0.0, 0.0, 0.0),
            formation_goal=pose(1.0, 0.0, 0.0),
            vehicles=[], allow_reverse=False)
        event = core.on_request(bad, publication_stamp=(1007, 0))
        self.assertEqual(event.status["node_state"], "REQUEST_REJECTED")
        self.assertEqual(event.status["status"], "INVALID")
        self.assertTrue(event.clear)
        self.assertIsNone(core.gate.published_plan_id)
        self.assertIsNone(core.snapshot)
        self.assertFalse(event.status["feasible"])

    def test_map_change_clears_publication(self):
        core = publishable_core()
        request = straight_request()
        _, _, first = publish(core, request, primitive(0, dx=2.0))
        self.assertTrue(first.status["trajectory_published"])

        changed = empty_grid()
        changed.occ[0, 0] = True
        event = core.on_map(changed, None, "map", stamp=(1008, 0))
        self.assertIsNotNone(event)
        self.assertEqual(event.status["node_state"], "MAP_CHANGED_CLEARED")
        self.assertTrue(event.clear)
        self.assertIsNone(core.gate.published_plan_id)
        self.assertFalse(event.status["trajectory_published"])
        self.assertEqual(event.status["status"], "INVALID")

    def test_map_version_is_pinned_between_request_and_seed(self):
        core = publishable_core()
        request = straight_request()
        request_event = core.on_request(
            request, publication_stamp=(1009, 0))
        self.assertEqual(request_event.status["node_state"],
                         "REQUEST_VALIDATED")
        first_signature = core.request_map_signature
        self.assertIsNotNone(first_signature)

        changed = empty_grid()
        changed.occ[0, 0] = True
        self.assertIsNone(core.on_map(
            changed, None, "map", stamp=(1010, 0)))
        seed = seed_message(request, (primitive(0, dx=2.0),))
        event = core.on_seed(seed, publication_stamp=(1011, 0))
        self.assertEqual(event.status["node_state"], "MAP_VERSION_MISMATCH")
        self.assertTrue(event.clear)
        self.assertFalse(event.status["trajectory_published"])
        self.assertIsNone(core.gate.published_plan_id)
        self.assertEqual(event.status["status"], "INVALID")

    def test_seed_before_request_is_cached_and_replayed(self):
        core = FormationNodeCore(pipeline_config=pipeline_config())
        request = straight_request()
        seed = seed_message(request, (primitive(0, dx=2.0),))
        cached = core.on_seed(seed, publication_stamp=(1012, 0))
        self.assertEqual(cached.status["node_state"], "SEED_CACHED")
        self.assertFalse(cached.clear)
        self.assertIsNone(cached.trajectory_fields)

        core.on_map(empty_grid(), None, "map", stamp=(1013, 0))
        request_event = core.on_request(
            request, publication_stamp=(1014, 0))
        self.assertEqual(request_event.status["node_state"],
                         "FEASIBLE_PUBLISHED")
        self.assertTrue(request_event.clear)
        self.assertTrue(request_event.status["trajectory_published"])
        self.assertEqual(core.gate.published_plan_id, request.plan_id)

    def test_request_before_map_pins_the_first_map_and_plans(self):
        core = FormationNodeCore(pipeline_config=pipeline_config())
        request = straight_request()
        request_event = core.on_request(
            request, publication_stamp=(1017, 0))
        self.assertEqual(request_event.status["node_state"],
                         "REQUEST_VALIDATED")
        self.assertIsNone(core.request_map_signature)
        self.assertIsNone(core.on_map(
            empty_grid(), None, "map", stamp=(1018, 0)))
        self.assertIsNotNone(core.request_map_signature)
        seed = seed_message(request, (primitive(0, dx=2.0),))
        event = core.on_seed(seed, publication_stamp=(1019, 0))
        self.assertEqual(event.status["node_state"], "FEASIBLE_PUBLISHED")
        self.assertTrue(event.status["trajectory_published"])

    def test_future_pending_seed_survives_an_older_request(self):
        core = FormationNodeCore(pipeline_config=pipeline_config())
        core.on_map(empty_grid(), None, "map", stamp=(1020, 0))
        request_41 = straight_request(plan_id=41)
        request_42 = straight_request(plan_id=42)
        seed_42 = seed_message(request_42, (primitive(0, dx=2.0),))
        cached = core.on_seed(seed_42, publication_stamp=(1021, 0))
        self.assertEqual(cached.status["node_state"], "SEED_CACHED")
        older = core.on_request(request_41, publication_stamp=(1022, 0))
        self.assertEqual(older.status["node_state"], "REQUEST_VALIDATED")
        self.assertIsNotNone(core.pending_seed)
        newer = core.on_request(request_42, publication_stamp=(1023, 0))
        self.assertEqual(newer.status["node_state"], "FEASIBLE_PUBLISHED")
        self.assertTrue(newer.status["trajectory_published"])
        self.assertEqual(newer.status["plan_id"], 42)

    def test_late_map_replays_the_cached_seed(self):
        core = FormationNodeCore(pipeline_config=pipeline_config())
        request = straight_request()
        core.on_request(request, publication_stamp=(1024, 0))
        seed = seed_message(request, (primitive(0, dx=2.0),))
        first = core.on_seed(seed, publication_stamp=(1025, 0))
        self.assertEqual(first.status["node_state"], "MAP_UNAVAILABLE")
        self.assertIsNotNone(core.pending_seed)
        self.assertIsNone(core.gate.published_plan_id)
        event = core.on_map(empty_grid(), None, "map", stamp=(1026, 0))
        self.assertIsNotNone(event)
        self.assertEqual(event.status["node_state"], "FEASIBLE_PUBLISHED")
        self.assertTrue(event.status["trajectory_published"])
        self.assertIsNone(core.pending_seed)

    def test_no_map_rejects_seed_and_never_publishes(self):
        core = FormationNodeCore(pipeline_config=pipeline_config())
        request = straight_request()
        request_event = core.on_request(
            request, publication_stamp=(1015, 0))
        self.assertEqual(request_event.status["node_state"],
                         "REQUEST_VALIDATED")
        seed = seed_message(request, (primitive(0, dx=2.0),))
        event = core.on_seed(seed, publication_stamp=(1016, 0))
        self.assertEqual(event.status["node_state"], "MAP_UNAVAILABLE")
        self.assertEqual(event.status["status"], "INVALID")
        self.assertFalse(event.status["trajectory_published"])
        self.assertIsNone(core.gate.published_plan_id)


if __name__ == "__main__":
    unittest.main()