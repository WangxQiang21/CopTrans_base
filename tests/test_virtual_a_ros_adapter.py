"""G5-A: the ROS-free virtual_rigid_a hook surface and the planner boundary.

These tests use the real frozen G4 scene inputs and the real autonomous G4
pipeline.  No reference trajectory is injected.  The adapter is exercised
through the same normalised ``FormationRequestSnapshot`` the ROS bridge
produces, and every rejection is asserted to be structured and non-mutating.
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts"))
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(ROOT / "tests"))

import virtual_a_g4_support as g4  # noqa: E402
import virtual_a_g5_support as g5  # noqa: E402
from fwis_v3_optimizer.continuous_geometry import ControlKnot  # noqa: E402
from fwis_v3_optimizer.formation import FormationPose  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    GoalPolicy,
    VirtualAContractError,
    research_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (  # noqa: E402
    CORRIDOR_DISPLAY_STATE,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (  # noqa: E402
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (  # noqa: E402
    plan_virtual_a,
)
from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    PLANNING_PUBLISHED_STATE,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (  # noqa: E402
    INITIAL_RESET_NOT_CONFIRMED,
    INVALID_INITIAL_CONTEXT,
    INVALID_PUBLICATION_DT,
    INVALID_PUBLICATION_STAMP,
    INVALID_SETTINGS,
    MAP_UNAVAILABLE,
    NONPARALLEL_INITIAL_HEADINGS,
    PUBLICATION_REQUEST_MISMATCH,
    PUBLICATION_SAMPLE_BUDGET,
    SOLVE_RESULT_MISMATCH,
    UNSUPPORTED_GOAL_HEADING,
    UNSUPPORTED_INITIAL_STATE,
    VirtualABackendHooks,
    VirtualABackendSettings,
    VirtualAInitialContext,
    publication_fields,
    request_from_snapshot,
    result_summary,
    solve,
    terminal_context_from_plan,
    verify_envelope_fields,
)
import formation_uniform_planner_node as planner_node  # noqa: E402
from formation_uniform_planner_node import (  # noqa: E402
    HOLD_END,
    IDLE,
    PLAYING,
    PLANNING,
    UniformPlannerNode,
)


S2 = "S2"


def _reject(testcase, reason, function, *args, **kwargs):
    with testcase.assertRaises(VirtualAContractError) as caught:
        function(*args, **kwargs)
    testcase.assertEqual(caught.exception.reason, reason)
    return caught.exception


def _eta_terminal_path(length=4.0, eta=0.1):
    """A path whose terminal has kappa=0 but eta!=0 and all headings equal."""
    profile = C2CurvatureProfile.from_knots([
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(float(length), 0.0, float(eta)),
    ])
    return VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile, g4.MODEL)


class _PipelineCases:
    """Process-wide real pipeline results; no injected reference."""
    grid = None
    motion_request = None
    motion_map_request = None
    motion_result = None
    hold_request = None
    hold_map_request = None
    hold_result = None

    @classmethod
    def build(cls):
        if cls.grid is not None:
            return
        cls.grid = g4.empty_grid()
        cls.motion_request, _, _ = g4.retime(
            g4.straight_path(10.0), cls.grid)
        cls.motion_map_request = g4.map_request(cls.motion_request, cls.grid)
        cls.motion_result = plan_virtual_a(cls.motion_map_request)
        assert cls.motion_result.status == "SUCCESS", \
            cls.motion_result.status
        cls.hold_request, _, _ = g4.retime(g4.hold_path(), cls.grid)
        cls.hold_map_request = g4.map_request(cls.hold_request, cls.grid)
        cls.hold_result = plan_virtual_a(cls.hold_map_request)
        assert cls.hold_result.status == "NO_MOTION", \
            cls.hold_result.status


class SettingsAndContextTest(unittest.TestCase):

    def test_default_settings_are_free_map(self):
        settings = VirtualABackendSettings()
        self.assertEqual(settings.goal_policy, "free")
        self.assertEqual(settings.policy, GoalPolicy.FREE)
        self.assertEqual(settings.frame_id, "map")
        self.assertGreaterEqual(settings.max_publication_samples, 1)

    def test_official_settings_policy_is_normalised(self):
        settings = VirtualABackendSettings(goal_policy="  STRAIGHT  ")
        self.assertEqual(settings.goal_policy, "straight")
        self.assertEqual(settings.policy, GoalPolicy.STRAIGHT)

    def test_invalid_settings_are_structured(self):
        for kwargs in ({"goal_policy": "mixed"},
                       {"goal_policy": 1},
                       {"frame_id": "odom"},
                       {"max_publication_samples": 0},
                       {"max_publication_samples": True}):
            with self.subTest(kwargs=kwargs):
                _reject(self, INVALID_SETTINGS, VirtualABackendSettings,
                        **kwargs)

    def test_initial_context_sources_are_ros_only(self):
        for source in ("offline_fixture", "manual", ""):
            with self.subTest(source=source):
                _reject(self, INVALID_INITIAL_CONTEXT,
                        VirtualAInitialContext, source,
                        "context-1", FormationPose(0, 0, 0))

    def test_verified_terminal_requires_headings(self):
        _reject(self, INVALID_INITIAL_CONTEXT, VirtualAInitialContext,
                "verified_terminal", "context-1",
                FormationPose(0, 0, 0))

    def test_context_serialises_sorted_headings_and_terminal(self):
        context = VirtualAInitialContext(
            source="verified_terminal", context_id="context-1",
            qf=FormationPose(1, 2, 0.3),
            headings={3: 0.3, 1: 0.3, 2: 0.3},
            terminal={"z": 1, "a": 2})
        document = context.as_dict()
        self.assertEqual([item["vehicle_id"]
                          for item in document["headings"]], [1, 2, 3])
        self.assertEqual(document["terminal"], {"a": 2, "z": 1})
        self.assertEqual(document["source"], "verified_terminal")


class RequestFromSnapshotTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.scene = g5.load_scene(S2)
        cls.settings_free = g5.settings_for(cls.scene, "free")
        cls.settings_straight = g5.settings_for(cls.scene, "straight")

    def _reset(self, qf):
        return VirtualAInitialContext(
            source="rviz_reset", context_id="virtual-a-reset-test-1", qf=qf)

    def test_free_request_uses_real_headings_shapes_and_offsets(self):
        start = FormationPose(0.0, 0.0, 0.0)
        headings = {1: 0.0, 2: 1e-8, 3: -1e-8}
        snapshot = g5.snapshot_for(
            self.scene, policy="free", start=start,
            start_headings=headings,
            goal_heading_free={1: True, 2: True, 3: True})
        map_request = request_from_snapshot(
            snapshot, self.scene["grid"], self.settings_free,
            self._reset(start))
        request = map_request.request
        self.assertEqual(request.plan_id, snapshot.plan_id)
        self.assertEqual(request.formation_id, snapshot.formation_id)
        self.assertEqual(request.goal_policy, GoalPolicy.FREE)
        self.assertEqual(request.initial.source, "rviz_reset")
        self.assertEqual(request.initial.context_id,
                         "virtual-a-reset-test-1")
        self.assertEqual(request.start_qf, start)
        self.assertEqual(request.goal_qf, snapshot.goal)
        self.assertEqual(request.start_headings, headings)
        self.assertEqual(request.model, snapshot.model)
        self.assertEqual(request.vehicle_shapes, snapshot.vehicle_shapes)
        self.assertTrue(all(snapshot.goal_heading_free.values()))
        self.assertEqual(map_request.grid, self.scene["grid"])
        self.assertEqual(map_request.planner_config,
                         self.settings_free.planner_config)
        self.assertEqual(map_request.timing_config,
                         self.settings_free.timing_config)
        self.assertEqual(map_request.collision_config,
                         self.settings_free.collision_config)

    def test_research_settings_reach_the_map_request_and_corridor_mode(self):
        config = research_planner_config()
        settings = VirtualABackendSettings(
            planner_config=config,
            timing_config=self.settings_free.timing_config,
            collision_config=self.settings_free.collision_config,
            goal_policy="free")
        start = FormationPose(0.0, 0.0, 0.0)
        snapshot = g5.snapshot_for(
            self.scene, policy="free", start=start,
            start_headings={1: 0.0, 2: 1e-8, 3: -1e-8},
            goal_heading_free={1: True, 2: True, 3: True})
        map_request = request_from_snapshot(
            snapshot, self.scene["grid"], settings, self._reset(start))
        self.assertIs(map_request.planner_config, config)
        self.assertEqual(map_request.planner_config.guidance_mode, "corridor")
        self.assertEqual(
            map_request.planner_config.corridor_enforcement, "hard")
        context = map_request.context()
        self.assertEqual(
            context["planner_config"]["guidance_mode"], "corridor")
        self.assertEqual(
            context["planner_config"]["corridor_enforcement"], "hard")

    def test_straight_request_accepts_all_fixed_compatible_headings(self):
        snapshot = g5.snapshot_for(
            self.scene, policy="straight",
            goal_heading_free={1: False, 2: False, 3: False})
        request = request_from_snapshot(
            snapshot, self.scene["grid"], self.settings_straight,
            self._reset(snapshot.start)).request
        self.assertEqual(request.goal_policy, GoalPolicy.STRAIGHT)
        self.assertEqual(request.start_headings,
                         dict(snapshot.start_headings))
        self.assertEqual(request.goal_qf, snapshot.goal)

    def test_missing_reset_is_rejected(self):
        snapshot = g5.snapshot_for(self.scene, policy="free")
        _reject(self, INITIAL_RESET_NOT_CONFIRMED,
                request_from_snapshot, snapshot, self.scene["grid"],
                self.settings_free, None)

    def test_missing_grid_is_rejected(self):
        snapshot = g5.snapshot_for(self.scene, policy="free")
        _reject(self, MAP_UNAVAILABLE, request_from_snapshot,
                snapshot, None, self.settings_free,
                self._reset(snapshot.start))

    def test_start_pose_outside_reset_tolerance_is_rejected(self):
        snapshot = g5.snapshot_for(
            self.scene, policy="free",
            start=FormationPose(1e-4, 0.0, 0.0))
        error = _reject(
            self, UNSUPPORTED_INITIAL_STATE, request_from_snapshot,
            snapshot, self.scene["grid"], self.settings_free,
            self._reset(self.scene["request"].start_qf))
        self.assertEqual(error.location["source"], "rviz_reset")

    def test_rviz_reset_rejects_nonparallel_real_headings(self):
        start = self.scene["request"].start_qf
        snapshot = g5.snapshot_for(
            self.scene, policy="free",
            start_headings={1: 0.2, 2: 0.0, 3: 0.0},
            goal_heading_free={1: True, 2: True, 3: True})
        error = _reject(
            self, NONPARALLEL_INITIAL_HEADINGS, request_from_snapshot,
            snapshot, self.scene["grid"], self.settings_free,
            self._reset(start))
        self.assertEqual(error.location["vehicle_id"], 1)
        self.assertEqual(error.location["context_id"],
                         "virtual-a-reset-test-1")

    def test_free_requires_every_goal_heading_free(self):
        snapshot = g5.snapshot_for(
            self.scene, policy="free",
            goal_heading_free={1: True, 2: False, 3: True})
        _reject(self, UNSUPPORTED_GOAL_HEADING, request_from_snapshot,
                snapshot, self.scene["grid"], self.settings_free,
                self._reset(snapshot.start))

    def test_straight_requires_every_goal_heading_fixed(self):
        snapshot = g5.snapshot_for(
            self.scene, policy="straight",
            goal_heading_free={1: False, 2: True, 3: False})
        _reject(self, UNSUPPORTED_GOAL_HEADING, request_from_snapshot,
                snapshot, self.scene["grid"], self.settings_straight,
                self._reset(snapshot.start))

    def test_straight_goal_heading_must_equal_goal_psi(self):
        goal_psi = self.scene["request"].goal_qf.psi
        snapshot = g5.snapshot_for(
            self.scene, policy="straight",
            goal_headings={1: goal_psi, 2: goal_psi + 1e-3,
                           3: goal_psi},
            goal_heading_free={1: False, 2: False, 3: False})
        error = _reject(
            self, UNSUPPORTED_GOAL_HEADING, request_from_snapshot,
            snapshot, self.scene["grid"], self.settings_straight,
            self._reset(snapshot.start))
        self.assertEqual(error.location["vehicle_id"], 2)

    def test_start_heading_ids_must_match_the_model(self):
        snapshot = g5.snapshot_for(
            self.scene, policy="free",
            start_headings={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
        _reject(self, "INVALID_SNAPSHOT", request_from_snapshot,
                snapshot, self.scene["grid"], self.settings_free,
                self._reset(snapshot.start))

    def test_verified_terminal_straight_continuation(self):
        request, result, _ = g4.retime(
            g4.straight_path(10.0), g4.empty_grid())
        plan = result.plan
        terminal = terminal_context_from_plan(
            plan, request, "virtual-a-terminal-test-1")
        self.assertTrue(terminal.compatible)
        snapshot = g5.snapshot_for(
            self.scene, policy="straight", start=terminal.qf,
            start_headings=terminal.headings,
            goal_heading_free={1: False, 2: False, 3: False})
        request = request_from_snapshot(
            snapshot, self.scene["grid"], self.settings_straight,
            terminal).request
        self.assertEqual(request.initial.source, "verified_terminal")
        self.assertEqual(request.initial.context_id,
                         "virtual-a-terminal-test-1")
        self.assertEqual(request.start_qf, terminal.qf)
        self.assertEqual(request.start_headings, terminal.headings)
        self.assertEqual(request.goal_policy, GoalPolicy.STRAIGHT)

    def test_verified_terminal_rejects_tampered_heading(self):
        request, result, _ = g4.retime(
            g4.straight_path(10.0), g4.empty_grid())
        plan = result.plan
        terminal = terminal_context_from_plan(
            plan, request, "virtual-a-terminal-test-1")
        headings = dict(terminal.headings)
        headings[2] += 1e-6
        snapshot = g5.snapshot_for(
            self.scene, policy="straight", start=terminal.qf,
            start_headings=headings,
            goal_heading_free={1: False, 2: False, 3: False})
        error = _reject(
            self, UNSUPPORTED_INITIAL_STATE, request_from_snapshot,
            snapshot, self.scene["grid"], self.settings_straight,
            terminal)
        self.assertEqual(error.location["source"], "verified_terminal")
        self.assertEqual(error.location["vehicle_id"], 2)
        self.assertEqual(error.location["terminal"]["t_s"],
                         terminal.terminal["t_s"])

    def test_incompatible_terminal_is_rejected_without_straightening(self):
        request, result, _ = g4.retime(
            _eta_terminal_path(), g4.empty_grid())
        plan = result.plan
        terminal = terminal_context_from_plan(
            plan, request, "virtual-a-terminal-eta")
        state = plan.state_at(plan.duration_s)
        self.assertFalse(terminal.compatible)
        self.assertAlmostEqual(state.vehicle(1).C_i, 0.1, places=12)
        self.assertAlmostEqual(
            state.vehicle(1).theta, state.qf.psi, places=12)
        self.assertGreater(
            max(abs(value) for value in state.vehicle(1).wheel_angles),
            1e-3)
        before = plan.to_model_json()
        snapshot = g5.snapshot_for(
            self.scene, policy="free", start=terminal.qf,
            start_headings=terminal.headings,
            goal_heading_free={1: True, 2: True, 3: True})
        error = _reject(
            self, UNSUPPORTED_INITIAL_STATE, request_from_snapshot,
            snapshot, self.scene["grid"], self.settings_free, terminal)
        self.assertEqual(error.location["source"], "verified_terminal")
        self.assertEqual(error.location["context_id"],
                         "virtual-a-terminal-eta")
        self.assertFalse(
            error.location["terminal"]["vehicles"][0]["eligible"])
        # The adapter never rewrites the authoritative terminal state.
        self.assertEqual(plan.to_model_json(), before)
        self.assertEqual(terminal.qf, state.qf)
        self.assertEqual(terminal.headings,
                         {vehicle.vehicle_id: vehicle.theta
                          for vehicle in state.vehicles})


class TerminalContextTest(unittest.TestCase):

    def test_straight_terminal_is_compatible(self):
        request, result, _ = g4.retime(
            g4.straight_path(10.0), g4.empty_grid())
        plan = result.plan
        context = terminal_context_from_plan(
            plan, request, "virtual-a-terminal-straight")
        self.assertTrue(context.compatible)
        self.assertEqual(context.source, "verified_terminal")
        self.assertEqual(context.context_id, "virtual-a-terminal-straight")
        self.assertEqual(context.qf, plan.state_at(plan.duration_s).qf)
        self.assertEqual(sorted(context.headings), [1, 2, 3])
        summary = context.terminal
        self.assertTrue(summary["all_stopped"])
        self.assertTrue(summary["all_parallel"])
        self.assertTrue(summary["all_wheels_straight"])
        for record in summary["vehicles"]:
            self.assertTrue(record["eligible"])
            self.assertTrue(all(record["checks"].values()))

    def test_kappa_zero_eta_nonzero_terminal_is_not_a_continuation(self):
        request, result, _ = g4.retime(
            _eta_terminal_path(), g4.empty_grid())
        plan = result.plan
        state = plan.state_at(plan.duration_s)
        context = terminal_context_from_plan(
            plan, request, "virtual-a-terminal-eta")
        self.assertFalse(context.compatible)
        summary = context.terminal
        self.assertTrue(summary["all_parallel"])
        self.assertTrue(summary["all_stopped"])
        self.assertFalse(summary["all_wheels_straight"])
        vehicle_one = summary["vehicles"][0]
        self.assertFalse(vehicle_one["checks"]["formation_curvature"])
        self.assertFalse(vehicle_one["checks"]["ideal_wheel_angles"])
        self.assertTrue(vehicle_one["checks"]["heading_parallel"])
        self.assertAlmostEqual(vehicle_one["C_i"], 0.1, places=12)
        self.assertAlmostEqual(
            state.vehicle(1).theta, state.qf.psi, places=12)
        self.assertIn("vehicle_id=1", context.detail)
        self.assertEqual(
            {record["vehicle_id"] for record in summary["vehicles"]},
            {1, 2, 3})

    def test_ramp_terminal_keeps_actual_not_straightened_state(self):
        request, result, _ = g4.retime(
            g4.ramp_path(4.0, 0.2), g4.empty_grid())
        plan = result.plan
        context = terminal_context_from_plan(
            plan, request, "virtual-a-terminal-ramp")
        self.assertFalse(context.compatible)
        state = plan.state_at(plan.duration_s)
        self.assertAlmostEqual(state.vehicle(1).C_i, 0.196116, places=5)
        self.assertGreater(abs(context.headings[1] - context.qf.psi), 0.1)
        self.assertFalse(context.terminal["all_wheels_straight"])

    def test_hold_terminal_is_compatible_and_stationary(self):
        request, result, _ = g4.retime(g4.hold_path(), g4.empty_grid())
        plan = result.plan
        context = terminal_context_from_plan(
            plan, request, "virtual-a-terminal-hold")
        self.assertTrue(context.compatible)
        self.assertEqual(context.terminal["t_s"], 0.0)
        for record in context.terminal["vehicles"]:
            self.assertEqual(record["v_i"], 0.0)
            self.assertEqual(record["omega_i"], 0.0)

    def test_invalid_plan_type_is_rejected(self):
        request = g4.make_request(FormationPose(0, 0, 0),
                                  FormationPose(0, 0, 0))
        _reject(self, "INVALID_PLAN", terminal_context_from_plan,
                object(), request, "context")


class PublicationFieldsTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _PipelineCases.build()
        for name in ("grid", "motion_request", "motion_map_request",
                     "motion_result", "hold_request", "hold_map_request",
                     "hold_result"):
            setattr(cls, name, getattr(_PipelineCases, name))
        cls.settings = VirtualABackendSettings()

    def _fields(self, result=None, snapshot=None, stamp=g5.STAMP, dt=0.02,
                settings=None):
        result = self.motion_result if result is None else result
        request = result.plan_request if hasattr(result, "plan_request") \
            else (self.motion_request if result is self.motion_result
                  else self.hold_request)
        snapshot = (g5.snapshot_from_request(request)
                    if snapshot is None else snapshot)
        return publication_fields(
            snapshot, result, stamp, dt,
            self.settings if settings is None else settings)

    def test_motion_fields_use_integer_nanosecond_times(self):
        fields = self._fields()
        plan = self.motion_result.plan
        times_ns = fields["times_ns"]
        self.assertEqual(tuple(fields["formation_poses"]).__len__(),
                         len(times_ns))
        self.assertGreater(len(times_ns), 100)
        self.assertEqual(times_ns[0], 0)
        self.assertEqual(times_ns, tuple(sorted(times_ns)))
        self.assertEqual(len(set(times_ns)), len(times_ns))
        for value in times_ns:
            self.assertIsInstance(value, int)
            self.assertGreaterEqual(value, 0)
        self.assertEqual(times_ns[-1], int(math.ceil(
            plan.duration_s * 1e9)))
        self.assertGreaterEqual(times_ns[-1] / 1e9, plan.duration_s)
        if times_ns[-1] > 1:
            self.assertLess((times_ns[-1] - 1) / 1e9, plan.duration_s)
        for value in times_ns[:-1]:
            if value:
                self.assertEqual(value % 20000000, 0)
        self.assertEqual(fields["stamp"], g5.STAMP)
        self.assertEqual(fields["frame_id"], "map")
        self.assertEqual(fields["plan_id"], self.motion_request.plan_id)
        self.assertEqual(fields["formation_id"],
                         self.motion_request.formation_id)
        self.assertEqual(fields["times_from_start"],
                         tuple(secs + nsecs * 1e-9 for secs, nsecs in
                               (divmod(value, 1000000000)
                                for value in times_ns)))
        for trajectory in fields["trajectories"]:
            self.assertEqual(trajectory["point_times_ns"], times_ns)
            self.assertEqual(len(trajectory["points"]), len(times_ns))

    def test_samples_are_evaluated_at_actual_quantised_times(self):
        fields = self._fields()
        plan = self.motion_result.plan
        times_ns = fields["times_ns"]
        times_s = fields["times_from_start"]
        for vehicle in fields["vehicles"]:
            vehicle_id = vehicle["vehicle_id"]
            points = next(
                item for item in fields["trajectories"]
                if item["vehicle_id"] == vehicle_id)["points"]
            for index in (0, 1, len(times_ns) // 3,
                          len(times_ns) - 2, len(times_ns) - 1):
                state = plan.state_at(times_s[index])
                vehicle_state = state.vehicle(vehicle_id)
                point = points[index]
                self.assertEqual(point[0], times_s[index])
                self.assertEqual(point[1], vehicle_state.x)
                self.assertEqual(point[2], vehicle_state.y)
                self.assertEqual(point[3], vehicle_state.theta)
                self.assertEqual(point[4], vehicle_state.v_i)
                self.assertEqual(point[5], vehicle_state.omega_i)
                self.assertEqual(point[6], 0)
                self.assertEqual(point[7], vehicle_state.C_i)
                self.assertEqual(fields["formation_poses"][index],
                                 (state.qf.x, state.qf.y, state.qf.psi))

    def test_vehicle_metadata_is_not_rewritten(self):
        fields = self._fields()
        plan = self.motion_result.plan
        request = self.motion_request
        self.assertEqual(tuple(item["vehicle_id"]
                               for item in fields["vehicles"]), (1, 2, 3))
        for item in fields["vehicles"]:
            vehicle_id = item["vehicle_id"]
            offset = next(offset for offset in request.model.offsets
                          if offset.vehicle_id == vehicle_id)
            shape = request.vehicle_shapes[vehicle_id]
            self.assertEqual(item["offset"], (offset.x, offset.y))
            self.assertEqual(item["body_length_m"], shape.length_m)
            self.assertEqual(item["body_width_m"], shape.width_m)
            self.assertEqual(
                item["start_center"],
                (*request.model.centers(request.start_qf)[vehicle_id],
                 request.start_headings[vehicle_id]))
            self.assertEqual(
                item["goal_center"],
                (*request.model.centers(request.goal_qf)[vehicle_id],
                 request.goal_qf.psi))
            self.assertTrue(item["goal_heading_free"])

    def test_no_motion_hold_publishes_one_frame_at_zero(self):
        fields = self._fields(result=self.hold_result)
        self.assertEqual(fields["times_ns"], (0,))
        self.assertEqual(fields["times_from_start"], (0.0,))
        self.assertEqual(len(fields["formation_poses"]), 1)
        for trajectory in fields["trajectories"]:
            self.assertEqual(len(trajectory["points"]), 1)
            self.assertEqual(trajectory["point_times_ns"], (0,))
            self.assertEqual(trajectory["points"][0][0], 0.0)

    def test_failed_result_cannot_be_published(self):
        request, result, _ = g4.retime(
            g4.straight_path(10.0), g4.empty_grid())
        snapshot = g5.snapshot_from_request(request)
        _reject(self, "INVALID_RESULT", publication_fields,
                snapshot, result, g5.STAMP, 0.02)

    def test_publication_stamp_must_be_a_nonzero_integer_pair(self):
        for stamp in ((0, 0), (0.0, 1), (-1, 0), (1, 1000000000),
                      (True, 0), "1s", None):
            with self.subTest(stamp=stamp):
                _reject(self, INVALID_PUBLICATION_STAMP,
                        publication_fields,
                        g5.snapshot_from_request(self.motion_request),
                        self.motion_result, stamp, 0.02)

    def test_publication_dt_must_be_positive_and_finite(self):
        for dt in (0.0, -0.02, float("nan"), float("inf"), None):
            with self.subTest(dt=dt):
                _reject(self, INVALID_PUBLICATION_DT,
                        publication_fields,
                        g5.snapshot_from_request(self.motion_request),
                        self.motion_result, g5.STAMP, dt)

    def test_publication_sample_budget_is_enforced(self):
        settings = VirtualABackendSettings(max_publication_samples=2)
        _reject(self, PUBLICATION_SAMPLE_BUDGET, publication_fields,
                g5.snapshot_from_request(self.motion_request),
                self.motion_result, g5.STAMP, 0.02, settings)

    def test_snapshot_goal_mismatch_is_rejected(self):
        snapshot = g5.snapshot_from_request(self.motion_request)
        from dataclasses import replace
        tampered = replace(
            snapshot, goal=FormationPose(snapshot.goal.x + 1e-6,
                                         snapshot.goal.y,
                                         snapshot.goal.psi))
        _reject(self, PUBLICATION_REQUEST_MISMATCH, publication_fields,
                tampered, self.motion_result, g5.STAMP, 0.02)

    def test_snapshot_start_heading_mismatch_is_rejected(self):
        snapshot = g5.snapshot_from_request(self.motion_request)
        from dataclasses import replace
        headings = dict(snapshot.start_headings)
        headings[1] += 1e-6
        tampered = replace(snapshot, start_headings=headings)
        _reject(self, PUBLICATION_REQUEST_MISMATCH, publication_fields,
                tampered, self.motion_result, g5.STAMP, 0.02)

    def test_snapshot_identity_and_shape_mismatches_are_rejected(self):
        from dataclasses import replace
        from fwis_v3_optimizer.formation_collision import VehicleShape
        snapshot = g5.snapshot_from_request(self.motion_request)
        shapes = dict(snapshot.vehicle_shapes)
        shapes[1] = VehicleShape(1.0, 0.500)
        variants = (
            replace(snapshot, plan_id=snapshot.plan_id + 1),
            replace(snapshot, formation_id="other"),
            replace(snapshot, vehicle_shapes=shapes),
        )
        for tampered in variants:
            with self.subTest(tampered=tampered):
                _reject(self, PUBLICATION_REQUEST_MISMATCH,
                        publication_fields, tampered,
                        self.motion_result, g5.STAMP, 0.02)


class SolveAndSummaryTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        _PipelineCases.build()
        for name in ("grid", "motion_request", "motion_map_request",
                     "motion_result", "hold_request", "hold_map_request",
                     "hold_result"):
            setattr(cls, name, getattr(_PipelineCases, name))

    def test_solve_rechecks_the_executable_identity(self):
        result = solve(self.hold_map_request)
        self.assertEqual(result.status, "NO_MOTION")
        self.assertTrue(result.executable)
        self.assertEqual(result.plan.model_sha256(), result.model_sha256)
        self.assertEqual(result.plan.request_sha256,
                         self.hold_map_request.request_sha256)
        self.assertEqual(result.planning_context_sha256,
                         result.plan.planning_context_sha256())

    def test_solve_rejects_a_mismatched_result(self):
        plan = self.hold_result.plan
        fake = SimpleNamespace(
            executable=True, plan=plan,
            model_sha256=plan.model_sha256(),
            planning_context_sha256="0" * 64,
            grid_sha256=self.hold_map_request.grid_sha256,
            request_sha256=self.hold_map_request.request_sha256)
        import fwis_v3_optimizer.formation_continuous_a.ros_adapter as \
            adapter_module
        with patch.object(adapter_module, "plan_virtual_a", return_value=fake):
            _reject(self, SOLVE_RESULT_MISMATCH, solve,
                    self.hold_map_request)

    def test_result_summary_is_strict_json_and_authoritative(self):
        summary = result_summary(self.motion_result)
        json.dumps(summary, allow_nan=False)
        self.assertEqual(summary["status"], "SUCCESS")
        self.assertTrue(summary["executable"])
        self.assertEqual(summary["certification"],
                         "virtual_a_geometry_dynamics_collision/v1")
        self.assertEqual(summary["request"]["plan_id"],
                         self.motion_request.plan_id)
        self.assertEqual(summary["request"]["goal_policy"], "FREE")
        self.assertEqual(summary["request"]["initial"]["source"],
                         "offline_fixture")
        self.assertEqual(summary["virtual_a_playback"]["backend"],
                         "virtual_rigid_a")
        self.assertIn("planning context only",
                      summary["collision_claim"])
        terminal = summary["terminal"]
        state = self.motion_result.plan.state_at(
            self.motion_result.plan.duration_s)
        self.assertEqual(terminal["t_s"], state.t)
        self.assertEqual(terminal["qf"], {
            "x": state.qf.x, "y": state.qf.y, "psi": state.qf.psi})
        self.assertEqual(len(terminal["vehicles"]), 3)
        for actual, expected in zip(terminal["vehicles"], state.vehicles):
            self.assertEqual(actual["vehicle_id"], expected.vehicle_id)
            self.assertEqual(actual["x"], expected.x)
            self.assertEqual(actual["y"], expected.y)
            self.assertEqual(actual["theta"], expected.theta)
            self.assertEqual(actual["C_i"], expected.C_i)
            self.assertEqual(actual["wheel_angles_rad"],
                             list(expected.wheel_angles))
        self.assertTrue(summary["audits"]["geometry"]["passed"])
        self.assertTrue(summary["audits"]["dynamics"]["passed"])

    def test_no_motion_summary_reports_a_hold(self):
        summary = result_summary(self.hold_result)
        self.assertEqual(summary["status"], "NO_MOTION")
        self.assertTrue(summary["executable"])
        self.assertEqual(summary["terminal"]["t_s"], 0.0)
        for vehicle in summary["terminal"]["vehicles"]:
            self.assertEqual(vehicle["v_i"], 0.0)

    def test_missing_result_summary_is_explicit(self):
        self.assertEqual(result_summary(None),
                         {"status": "NONE", "executable": False})


class BackendHooksTest(unittest.TestCase):

    def test_hooks_expose_exactly_the_five_operations(self):
        settings = VirtualABackendSettings(goal_policy="straight")
        hooks = VirtualABackendHooks(settings)
        self.assertEqual(hooks.backend, "virtual_rigid_a")
        self.assertEqual(hooks.model_version, "virtual_rigid_a.timed.v1")
        self.assertEqual(hooks.semantic_version,
                         "virtual_rigid_a.playback.v1")
        self.assertEqual(hooks.certification,
                         "virtual_a_geometry_dynamics_collision/v1")
        self.assertIs(hooks.settings, settings)
        self.assertEqual(hooks.settings.goal_policy, "straight")

    def test_hooks_without_settings_use_defaults(self):
        hooks = VirtualABackendHooks()
        self.assertEqual(hooks.settings.goal_policy, "free")
        self.assertEqual(hooks.settings.frame_id, "map")


class _NodeHarness(unittest.TestCase):
    DIGEST = "a" * 64
    FORMATION = "G4_S2"
    PLAN_ID = 9002

    def setUp(self):
        _PipelineCases.build()
        self.events = []
        self.planning_debug = []
        import formation_uniform_planner_node
        self.module = formation_uniform_planner_node

    def _node(self, *, state=PLAYING, backend_hooks=None):
        node = UniformPlannerNode.__new__(UniformPlannerNode)
        node._lock = threading.RLock()
        node._state = state
        node._state_plan = self.PLAN_ID
        node._active_model_sha256 = self.DIGEST
        node._active_request_sha256 = "b" * 64
        node._active_formation_id = self.FORMATION
        node._active_status_word = "FEASIBLE"
        node._playback_seen = False
        node._playback_started_wall = None
        node._published_wall = 123.0
        node._published_stamp = (1, 2)
        node._published_duration = 1.0
        node._last_status = None
        node._last_quality = None
        node._requests_seen = 1
        node._busy_rejections = 0
        node._last_accepted_plan = self.PLAN_ID - 1
        node._reset_serial = 0
        node._initial_context = None
        node._initial_reset_ready = False
        node._worker_active = False
        node._generation = 0
        node._plan_started_wall = 1.0
        node._virtual_request = None
        node._last_result = None
        node._grid = object()
        node._map_signature = "map-sha"
        node._request_map_signature = "map-sha"
        node._map_frame_id = "map"
        node._map_error = None
        node._frame_id = "map"
        node.frame_id = "map"
        node.planning_timeout_s = 75.0
        node._backend_hooks = backend_hooks
        node.backend = ("virtual_rigid_a" if backend_hooks is not None
                        else "uniform_v02")
        node.model_version = getattr(
            backend_hooks, "model_version", "uniform_v02.timed.v1")
        node.semantic_version = getattr(
            backend_hooks, "semantic_version", "uniform_v02.playback.v1")
        node._next_context_id = lambda kind: \
            UniformPlannerNode._next_context_id(node, kind)
        node._publish_job_state = lambda reason="", detail="": \
            self.events.append(("job", node._state, reason))
        node._publish_status = lambda document: \
            self.events.append(("status", document))
        node._publish_quality = lambda document: \
            self.events.append(("quality", document))
        node.planning_debug_pub = SimpleNamespace(
            publish=self.planning_debug.append)

        def base_status(node_state, status, reason, detail, **kwargs):
            document = {
                "node_state": node_state, "status": status,
                "reason": reason, "detail": detail,
                "job_state": node._state, "plan_id": kwargs.get(
                    "plan_id", node._state_plan),
                "formation_id": kwargs.get("formation_id"),
                "feasible": bool(kwargs.get("feasible")),
                "executable": bool(kwargs.get("executable")),
                "trajectory_published": bool(kwargs.get("published")),
                "playback_confirmed": bool(node._playback_seen),
                "model_sha256": node._active_model_sha256,
                "request_sha256": node._active_request_sha256,
                "busy_rejections": node._busy_rejections,
            }
            document.update(kwargs.get("extra") or {})
            return document

        def base_quality(event, status, reason, detail, **kwargs):
            document = base_status("QUALITY", status, reason, detail,
                                   **kwargs)
            document["schema"] = "formation_quality_report/v1"
            document["event"] = event
            return document

        node._base_status = base_status
        node._base_quality = base_quality
        return node

    def _deliver(self, node, **overrides):
        report = {
            "state": "HOLD_END",
            "plan_id": node._state_plan,
            "formation_id": self.FORMATION,
            "backend": "virtual_rigid_a",
            "model_sha256": self.DIGEST,
            "executable": False,
        }
        report.update(overrides)
        with patch.object(self.module.rospy, "logwarn_throttle"), \
                patch.object(self.module.rospy, "loginfo"), \
                patch.object(self.module.rospy, "logerr"):
            node.on_playback_status(
                SimpleNamespace(data=json.dumps(report)))

    def _statuses(self):
        return [event[1] for event in self.events
                if event[0] == "status"]

    def _jobs(self):
        return [event for event in self.events if event[0] == "job"]


class PlannerBusyAndResetTest(_NodeHarness):

    def test_busy_request_rejected_without_consuming_state(self):
        node = self._node(state=PLAYING,
                          backend_hooks=VirtualABackendHooks())
        active = (node._state, node._state_plan, node._active_model_sha256,
                  node._initial_context)
        message = SimpleNamespace(plan_id=9003, formation_id="G4_S3")
        with patch.object(self.module.rospy, "logwarn"):
            node.on_request(message)
        self.assertEqual(
            (node._state, node._state_plan, node._active_model_sha256,
             node._initial_context), active)
        self.assertEqual(node._busy_rejections, 1)
        status = self._statuses()[-1]
        self.assertEqual(status["node_state"], "BUSY_REJECTED")
        self.assertEqual(status["status"], "REJECTED")
        self.assertTrue(status["busy_rejected"])
        self.assertEqual(status["rejected_plan_id"], 9003)
        self.assertEqual(status["active_plan_id"], self.PLAN_ID)
        self.assertEqual(status["active_job_state"], PLAYING)
        self.assertEqual(len(self._jobs()), 0)
        self.assertEqual([kind for kind, _ in self.events],
                         ["status", "quality"])

    def test_worker_active_rejects_request_even_when_idle(self):
        node = self._node(state=IDLE,
                          backend_hooks=VirtualABackendHooks())
        node._worker_active = True
        with patch.object(self.module.rospy, "logwarn"):
            node.on_request(SimpleNamespace(plan_id=9003,
                                           formation_id="G4_S3"))
        self.assertEqual(node._busy_rejections, 1)
        self.assertEqual(self._statuses()[-1]["reason"], "BUSY")

    def _pose(self, x=1.0, y=2.0, yaw=0.3, frame="map",
              roll=0.0, pitch=0.0):
        half_yaw = yaw / 2.0
        if roll or pitch:
            # Any non-planar quaternion is enough for the rejection path.
            qx, qy, qz, qw = (math.sin(roll / 2.0), math.sin(pitch / 2.0),
                              math.sin(half_yaw), math.cos(roll / 2.0))
        else:
            qx, qy, qz, qw = 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame),
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=0.0),
                orientation=SimpleNamespace(
                    x=qx, y=qy, z=qz, w=qw))))

    def test_valid_reset_records_a_new_initial_context(self):
        node = self._node(state=IDLE,
                          backend_hooks=VirtualABackendHooks())
        with patch.object(self.module.rospy, "loginfo"), \
                patch.object(self.module.rospy, "logwarn"):
            node.on_initial_pose(self._pose())
        context = node._initial_context
        self.assertIsNotNone(context)
        self.assertEqual(context.source, "rviz_reset")
        self.assertEqual(context.context_id,
                         "virtual-a-reset-9002-1")
        self.assertTrue(node._initial_reset_ready)
        status = self._statuses()[-1]
        self.assertEqual(status["node_state"], "RESET_CONFIRMED")
        self.assertEqual(status["status"], "IDLE")
        self.assertTrue(status["reset_consumed"])
        self.assertEqual(status["initial_context_id"],
                         context.context_id)
        self.assertAlmostEqual(status["initial_pose"]["x"], 1.0)
        self.assertAlmostEqual(status["initial_pose"]["y"], 2.0)
        self.assertAlmostEqual(status["initial_pose"]["yaw"], 0.3)
        self.assertEqual(len(self._jobs()), 1)

    def test_nonplanar_reset_is_rejected(self):
        node = self._node(state=IDLE,
                          backend_hooks=VirtualABackendHooks())
        with patch.object(self.module.rospy, "logwarn"):
            node.on_initial_pose(self._pose(roll=0.2))
        self.assertIsNone(node._initial_context)
        self.assertFalse(node._initial_reset_ready)
        self.assertEqual(self._statuses()[-1]["reason"],
                         "RESET_POSE_REJECTED")

    def test_wrong_frame_reset_is_rejected(self):
        node = self._node(state=IDLE,
                          backend_hooks=VirtualABackendHooks())
        with patch.object(self.module.rospy, "logwarn"):
            node.on_initial_pose(self._pose(frame="odom"))
        self.assertIsNone(node._initial_context)
        self.assertEqual(self._statuses()[-1]["reason"],
                         "RESET_FRAME_REJECTED")

    def test_busy_reset_does_not_replace_the_confirmed_context(self):
        node = self._node(state=PLAYING,
                          backend_hooks=VirtualABackendHooks())
        existing = VirtualAInitialContext(
            source="rviz_reset", context_id="existing",
            qf=FormationPose(0, 0, 0))
        node._initial_context = existing
        node._initial_reset_ready = True
        with patch.object(self.module.rospy, "logwarn"):
            node.on_initial_pose(self._pose())
        self.assertIs(node._initial_context, existing)
        self.assertEqual(node._busy_rejections, 1)
        status = self._statuses()[-1]
        self.assertEqual(status["node_state"], "RESET_REJECTED")
        self.assertEqual(status["reason"], "BUSY")
        self.assertFalse(status["reset_consumed"])

    def test_old_backend_ignores_initialpose(self):
        node = self._node(state=IDLE, backend_hooks=None)
        node.on_initial_pose(self._pose())
        self.assertIsNone(node._initial_context)
        self.assertEqual(self.events, [])


class PlannerFeedbackTest(_NodeHarness):

    def test_wrong_digest_terminal_does_not_release_busy(self):
        node = self._node(backend_hooks=VirtualABackendHooks())
        self._deliver(node, model_sha256="c" * 64)
        self.assertEqual(node._state, PLAYING)
        self.assertFalse(node._playback_seen)
        self.assertEqual(self.events, [])

    def test_missing_identity_terminal_does_not_release_busy(self):
        for missing in ("model_sha256", "formation_id", "backend"):
            node = self._node(backend_hooks=VirtualABackendHooks())
            report = {"state": "HOLD_END", "plan_id": self.PLAN_ID,
                      "formation_id": self.FORMATION,
                      "backend": "virtual_rigid_a",
                      "model_sha256": self.DIGEST}
            report.pop(missing)
            with patch.object(self.module.rospy, "logwarn_throttle"), \
                    patch.object(self.module.rospy, "loginfo"), \
                    patch.object(self.module.rospy, "logerr"):
                node.on_playback_status(
                    SimpleNamespace(data=json.dumps(report)))
            self.assertEqual(node._state, PLAYING, missing)
            self.assertEqual(self.events, [], missing)

    def test_prepublication_feedback_cannot_confirm(self):
        node = self._node(state=PLANNING,
                          backend_hooks=VirtualABackendHooks())
        node._active_model_sha256 = None
        self._deliver(node)
        self.assertEqual(node._state, PLANNING)
        self.assertFalse(node._playback_seen)
        self.assertEqual(self.events, [])

    def test_matching_terminal_releases_busy_and_records_straight_terminal(self):
        request, result, _ = g4.retime(
            g4.straight_path(10.0), g4.empty_grid())
        node = self._node(state=PLAYING,
                          backend_hooks=VirtualABackendHooks())
        node._state_plan = request.plan_id
        node._active_formation_id = request.formation_id
        node._active_model_sha256 = result.plan.model_sha256()
        node._virtual_request = request
        node._last_result = SimpleNamespace(plan=result.plan)
        self._deliver(
            node,
            plan_id=node._state_plan,
            formation_id=node._active_formation_id,
            model_sha256=node._active_model_sha256)
        self.assertEqual(node._state, HOLD_END)
        self.assertTrue(node._playback_seen)
        self.assertFalse(node._initial_reset_ready)
        terminal = node._initial_context
        self.assertIsNotNone(terminal)
        self.assertEqual(terminal.source, "verified_terminal")
        self.assertEqual(
            terminal.context_id,
            "virtual-a-terminal-{}-1".format(node._state_plan))
        self.assertTrue(terminal.compatible)
        status = self._statuses()[-1]
        self.assertEqual(status["node_state"], "HOLD_END")
        self.assertEqual(status["status"], "FEASIBLE")
        self.assertTrue(status["terminal"])
        self.assertTrue(status["terminal_compatible"])
        self.assertEqual(status["terminal_context_id"],
                         terminal.context_id)
        self.assertEqual(len(self._jobs()), 1)

    def test_matching_terminal_records_incompatible_terminal_truthfully(self):
        request, result, _ = g4.retime(
            _eta_terminal_path(), g4.empty_grid())
        node = self._node(state=PLAYING,
                          backend_hooks=VirtualABackendHooks())
        node._virtual_request = request
        node._last_result = SimpleNamespace(
            plan=result.plan, executable=True,
            model_sha256=result.plan.model_sha256())
        node._state_plan = request.plan_id
        node._active_formation_id = request.formation_id
        node._active_model_sha256 = result.plan.model_sha256()
        self._deliver(
            node, plan_id=request.plan_id,
            formation_id=request.formation_id,
            model_sha256=node._active_model_sha256)
        terminal = node._initial_context
        self.assertFalse(terminal.compatible)
        self.assertIn("vehicle_id=1", terminal.detail)
        status = self._statuses()[-1]
        self.assertFalse(status["terminal_compatible"])
        self.assertFalse(
            status["terminal_state"]["vehicles"][0]["eligible"])

    def test_hold_end_without_a_virtual_plan_keeps_old_behaviour(self):
        node = self._node(backend_hooks=None)
        node.backend = "uniform_v02"
        node._active_model_sha256 = self.DIGEST
        node._active_formation_id = "uniform"
        self._deliver(node, backend="uniform_v02",
                      formation_id="uniform")
        self.assertEqual(node._state, HOLD_END)
        self.assertIsNone(node._initial_context)
        self.assertNotIn("terminal_context_id", self._statuses()[-1])


class JobDocumentTest(_NodeHarness):

    def test_virtual_job_document_exposes_reset_and_worker_state(self):
        hooks = VirtualABackendHooks(
            VirtualABackendSettings(goal_policy="straight"))
        node = self._node(state=IDLE, backend_hooks=hooks)
        node._initial_context = VirtualAInitialContext(
            source="rviz_reset", context_id="virtual-a-reset-9002-1",
            qf=FormationPose(0, 0, 0))
        node._initial_reset_ready = True
        node._worker_active = True
        document = UniformPlannerNode._job_document(node)
        self.assertEqual(document["backend"], "virtual_rigid_a")
        self.assertTrue(document["initial_reset_ready"])
        self.assertEqual(document["initial_context_id"],
                         "virtual-a-reset-9002-1")
        self.assertEqual(document["initial_context_source"], "rviz_reset")
        self.assertTrue(document["initial_compatible"])
        self.assertTrue(document["worker_active"])
        self.assertEqual(document["planning_window_s"], 75.0)
        self.assertEqual(document["virtual_a_goal_policy"], "straight")

    def test_old_job_document_is_unchanged(self):
        node = self._node(state=IDLE, backend_hooks=None)
        node.backend = "uniform_v02"
        document = UniformPlannerNode._job_document(node)
        for key in ("initial_reset_ready", "initial_context_id",
                    "initial_context_source", "initial_compatible",
                    "worker_active", "planning_window_s",
                    "virtual_a_goal_policy"):
            self.assertNotIn(key, document)


class CorridorDisplayConstructionTest(_NodeHarness):
    """Progressive corridor display comes only from the current admission."""

    def _display(self, **overrides):
        display = {
            "state": CORRIDOR_DISPLAY_STATE,
            "plan_id": self.PLAN_ID,
            "formation_id": self.FORMATION,
            "backend": "virtual_rigid_a",
            "generation": 0,
            "grid_sha256": "c" * 64,
            "planning_context_sha256": "b" * 64,
            "model_sha256": None,
            "chain_digest": "d" * 64,
            "regions": [{"region_id": 0, "vertices": [[0, 0], [1, 0], [1, 1]]}],
        }
        display.update(overrides)
        return display

    def _view(self, displays):
        return SimpleNamespace(
            search={"corridor_displays": displays},
            grid_sha256="c" * 64,
            planning_context_sha256="b" * 64,
            model_sha256=None)

    def _corridor_node(self):
        node = self._node(backend_hooks=VirtualABackendHooks())
        node._virtual_admission = {
            "formation_id": self.FORMATION,
            "request_sha256": "e" * 64,
            "grid_sha256": "c" * 64,
            "planning_context_sha256": "b" * 64,
        }
        node._progress_layers = {}
        node._progress_display_errors = 0
        node._progress_display_disabled = set()
        snapshot = SimpleNamespace(formation_id=self.FORMATION)
        node._virtual_snapshot = snapshot
        return node, snapshot

    def _build(self, node, snapshot, displays):
        identity = node._planning_identity(
            0, self.PLAN_ID, snapshot, PLANNING)
        self.assertIsNotNone(identity)
        return node._corridor_displays(
            0, self.PLAN_ID, snapshot, self._view(displays), identity)

    def test_matching_display_fields_are_returned(self):
        node, snapshot = self._corridor_node()
        checked = self._build(node, snapshot, [self._display()])
        self.assertEqual(len(checked), 1)
        self.assertEqual(checked[0]["state"], CORRIDOR_DISPLAY_STATE)
        self.assertEqual(checked[0]["plan_id"], self.PLAN_ID)
        self.assertEqual(node._progress_display_errors, 0)

    def test_stale_or_malformed_display_clears_only_that_layer(self):
        node, snapshot = self._corridor_node()
        for display in (
                self._display(plan_id=self.PLAN_ID + 1),
                self._display(grid_sha256="f" * 64),
                self._display(state="FEASIBLE"),
                {"state": CORRIDOR_DISPLAY_STATE},
                []):
            with self.subTest(display=display):
                node._progress_display_errors = 0
                self.assertIsNone(self._build(node, snapshot, display))
                self.assertEqual(node._progress_display_errors, 1)


class VirtualEnvelopePublicationTest(unittest.TestCase):
    """The real plan is enveloped with the admitted identity, not guessed."""

    @classmethod
    def setUpClass(cls):
        cls.case = g5.solved("S2", "straight")
        cls.snapshot = cls.case["snapshot"]
        cls.request = cls.case["map_request"].request
        cls.admission = g5.admission_for(cls.case)

    def test_success_envelope_reuses_the_admitted_identity(self):
        published = []
        failures = []
        planning_debug = []
        order = []
        node = UniformPlannerNode.__new__(UniformPlannerNode)
        hooks = VirtualABackendHooks(self.case["settings"])
        node._backend_hooks = hooks
        node.backend = hooks.backend
        node.model_version = hooks.model_version
        node.semantic_version = hooks.semantic_version
        node.certification = hooks.certification
        node.publication_dt = 0.02
        node.frame_id = "map"
        node._state = PLANNING
        node._state_plan = self.request.plan_id
        node._virtual_request = self.request
        node._virtual_context = self.case["context"]
        node._virtual_grid_sha256 = self.case["map_request"].grid_sha256
        node._virtual_admission = self.admission
        node._publication_delay_s = None
        stamps = iter([(1000, 0), (1000, 10000000)])
        node._now = lambda: next(stamps)
        def record_envelope(message):
            published.append(message)
            order.append(("envelope", message))

        def record_planning(message):
            planning_debug.append(message)
            order.append(("planning", message))

        node.playback_plan_pub = SimpleNamespace(publish=record_envelope)
        node.planning_debug_pub = SimpleNamespace(publish=record_planning)
        node._publish_status = lambda document: None
        node._publish_quality = lambda document: None
        node._base_status = lambda *args, **kwargs: dict(kwargs)
        node._base_quality = lambda *args, **kwargs: dict(kwargs)
        node._finish_virtual_failure = (
            lambda *args, **kwargs: failures.append((args, kwargs)))
        result = self.case["result"]
        node._finish_virtual_success(
            0, self.request.plan_id, self.snapshot, result, 1.0)
        self.assertEqual(failures, [])
        self.assertEqual(len(published), 1)
        envelope = published[0]
        self.assertEqual(envelope.plan_id, self.request.plan_id)
        self.assertEqual(envelope.formation_id,
                         self.request.formation_id)
        self.assertEqual(envelope.model_sha256, result.model_sha256)
        self.assertEqual(envelope.model_json,
                         result.plan.to_model_json())
        self.assertEqual(envelope.certification, hooks.certification)
        stamp = (int(envelope.header.stamp.secs),
                 int(envelope.header.stamp.nsecs))
        self.assertEqual(stamp, (1000, 10000000))
        self.assertEqual(
            stamp, (int(envelope.trajectory.header.stamp.secs),
                    int(envelope.trajectory.header.stamp.nsecs)))
        # PUBLISHED is the ROS node's statement after the atomic envelope
        # exists.  The pipeline itself never emits it, and the document must
        # not precede the envelope on the wire.  This result has no C2
        # corridor round, so the corridor layer is cleared explicitly rather
        # than leaving an older display latched.
        self.assertEqual(
            [kind for kind, _ in order], ["envelope", "planning"])
        self.assertEqual(len(planning_debug), 1)
        document = json.loads(planning_debug[0].data)
        self.assertEqual(document["state"], PLANNING_PUBLISHED_STATE)
        self.assertEqual(document["stage"], PLANNING_PUBLISHED_STATE)
        self.assertEqual(document["stage_index"], 8)
        self.assertEqual(document["model_sha256"], result.model_sha256)
        self.assertEqual(document["plan_id"], self.request.plan_id)
        self.assertEqual(document["generation"], 0)
        self.assertIsNone(document["corridors"])


class NodeMainResearchEntryTest(unittest.TestCase):
    """The ROS params in main() reach hooks.settings.planner_config."""

    def _run_main(self, params):
        def get_param(name, default=None):
            return params.get(name, default)

        with patch.object(planner_node.rospy, "init_node"), \
                patch.object(planner_node.rospy, "get_param",
                             side_effect=get_param), \
                patch.object(planner_node.rospy, "spin"), \
                patch.object(planner_node.rospy, "loginfo") as loginfo, \
                patch.object(planner_node.rospy, "logfatal") as logfatal, \
                patch.object(planner_node, "UniformPlannerNode") as node_cls:
            planner_node.main()
        return node_cls, loginfo, logfatal

    def test_main_builds_the_validated_research_config(self):
        node_cls, loginfo, _logfatal = self._run_main({
            "~backend": "virtual_rigid_a",
            "~virtual_a_guidance_mode": " Corridor ",
            "~virtual_a_corridor_enforcement": "HARD",
        })
        hooks = node_cls.call_args.kwargs["backend_hooks"]
        config = hooks.settings.planner_config
        self.assertEqual(config.guidance_mode, "corridor")
        self.assertEqual(config.corridor_enforcement, "hard")
        logged = loginfo.call_args_list[0].args
        self.assertIn("validated planner config", logged[0])
        self.assertEqual(logged[1], "corridor")
        self.assertEqual(logged[2], "hard")

    def test_main_fails_closed_on_unknown_values(self):
        for params in (
                {"~backend": "virtual_rigid_a",
                 "~virtual_a_guidance_mode": "sideways"},
                {"~backend": "virtual_rigid_a",
                 "~virtual_a_corridor_enforcement": "loose"}):
            with self.assertRaises(VirtualAContractError) as raised:
                self._run_main(params)
            self.assertEqual(raised.exception.reason, "INVALID_CONFIG")


if __name__ == "__main__":
    unittest.main()