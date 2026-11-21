"""G5-A: verified virtual_rigid_a playback frames, markers and status.

The envelope is built from the real frozen S2 scene through the real G4
pipeline.  Every published sample, every interpolated playback frame, every
marker position and every status claim is compared with the authoritative
timed model (or rejected with a structured error).  No test injects a
reference trajectory.
"""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(ROOT / "tests"))

import virtual_a_g4_support as g4  # noqa: E402
import virtual_a_g5_support as g5  # noqa: E402
from fwis_v3_optimizer.formation import FormationPose  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    FORMATION_CORRIDOR_COMPILER_VERSION,
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    FORMATION_CORRIDOR_TEMPLATE_SCHEMA,
    GoalPolicy,
    VirtualAContractError,
    request_fingerprint,
    request_payload,
    research_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (  # noqa: E402
    formation_template_from_payload,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (  # noqa: E402
    plan_virtual_a,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (  # noqa: E402
    solve,
    verify_envelope_fields,
)
from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    ACKERMANN,
    HOLD_END,
    HOLD_START,
    PLAYING,
    PlaybackValidationError,
    _verify_virtual_time_axis,
    build_markers,
    frame_at,
    status_document,
    terminal_errors,
    vehicle_namespace,
    virtual_a_request_from_raw_request,
)


class VirtualTimeAxisTest(unittest.TestCase):
    def test_long_ros_duration_round_trips_without_rounding_away_a_nanosecond(self):
        times = (0.0, 64 + 20000000 / 1e9)
        nanoseconds = (0, 64020000000)
        fields = {"times_ns": nanoseconds, "trajectories": (
            {"vehicle_id": 1, "point_times_ns": nanoseconds},)}
        self.assertEqual(_verify_virtual_time_axis(fields, times), nanoseconds)
        self.assertEqual(_verify_virtual_time_axis({}, times), nanoseconds)
        fields["trajectories"][0]["point_times_ns"] = (0, 64020000001)
        with self.assertRaisesRegex(PlaybackValidationError, "point_times_ns"):
            _verify_virtual_time_axis(fields, times)
        with self.assertRaisesRegex(PlaybackValidationError, "integer-nanosecond"):
            _verify_virtual_time_axis({}, (0.0, 64.0200000004))


class EnvelopeTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.case = g5.solved("S2", "straight")
        cls.envelope = g5.envelope_for(cls.case)
        cls.verified = verify_envelope_fields(cls.envelope)
        cls.trajectory = cls.verified.trajectory
        cls.plan = cls.verified.plan
        cls.fields = cls.envelope["trajectory_fields"]
        cls.digest = cls.envelope["model_sha256"]
        cls.plan_id = cls.envelope["plan_id"]
        cls.formation_id = cls.envelope["formation_id"]

    def _mutable_fields(self):
        return json.loads(json.dumps(self.fields))

    def _verify(self, *, fields=None, drop=(), context=None,
                admission=None, request="default", **overrides):
        envelope = dict(self.envelope)
        envelope["trajectory_fields"] = (
            self._mutable_fields() if fields is None else fields)
        envelope["expected_context"] = copy.deepcopy(
            self.envelope["expected_context"] if context is None else context)
        envelope["expected_admission"] = copy.deepcopy(
            self.envelope["expected_admission"]
            if admission is None else admission)
        if request != "default":
            envelope["expected_request"] = request
        for key in drop:
            envelope.pop(key, None)
        envelope.update(overrides)
        return verify_envelope_fields(envelope)

    def _reject(self, reason=None, **kwargs):
        with self.assertRaises(VirtualAContractError) as caught:
            self._verify(**kwargs)
        self.assertTrue(caught.exception.reason)
        if reason is not None:
            self.assertEqual(caught.exception.reason, reason)
        return caught.exception

    def test_authoritative_envelope_is_accepted(self):
        self.assertEqual(self.verified.backend, "virtual_rigid_a")
        self.assertEqual(self.verified.semantic_version,
                         "virtual_rigid_a.playback.v1")
        self.assertEqual(self.verified.certification,
                         "virtual_a_geometry_dynamics_collision/v1")
        self.assertEqual(self.verified.model_sha256, self.digest)
        self.assertEqual(self.trajectory.model_sha256, self.digest)
        self.assertEqual(self.trajectory.backend, "virtual_rigid_a")
        self.assertIs(self.trajectory.virtual_a_plan, self.plan)
        self.assertGreaterEqual(
            self.trajectory.sample_count, 100)
        json.dumps(self.fields, allow_nan=False)

    def test_common_time_axis_is_integer_nanoseconds(self):
        times_ns = self.fields["times_ns"]
        self.assertEqual(
            tuple(times_ns), tuple(int(round(value * 1e9))
                                   for value in
                                   self.fields["times_from_start"]))
        self.assertEqual(times_ns[0], 0)
        self.assertEqual(tuple(times_ns), tuple(sorted(set(times_ns))))
        self.assertGreaterEqual(times_ns[-1] / 1e9,
                                self.plan.duration_s)
        self.assertLess((times_ns[-1] - 1) / 1e9,
                        self.plan.duration_s)
        for trajectory in self.fields["trajectories"]:
            self.assertEqual(trajectory["point_times_ns"], times_ns)

    def test_every_published_sample_matches_the_model(self):
        times_s = self.fields["times_from_start"]
        for trajectory in self.fields["trajectories"]:
            vehicle_id = trajectory["vehicle_id"]
            for index, point in enumerate(trajectory["points"]):
                state = self.plan.state_at(times_s[index])
                vehicle = state.vehicle(vehicle_id)
                self.assertEqual(point[0], times_s[index])
                self.assertEqual(point[1], vehicle.x)
                self.assertEqual(point[2], vehicle.y)
                self.assertEqual(point[3], vehicle.theta)
                self.assertEqual(point[4], vehicle.v_i)
                self.assertEqual(point[5], vehicle.omega_i)
                self.assertEqual(point[6], ACKERMANN)
                self.assertEqual(point[7], vehicle.C_i)
                self.assertEqual(
                    self.fields["formation_poses"][index],
                    (state.qf.x, state.qf.y, state.qf.psi))

    def test_integer_nanosecond_arrays_are_checked(self):
        fields = self._mutable_fields()
        fields["times_ns"][-1] += 1
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["times_ns"].pop()
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["trajectories"][0]["point_times_ns"][0] = 1
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["trajectories"][0]["point_times_ns"].pop()
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["times_from_start"][5] = fields["times_from_start"][4]
        fields["times_ns"][5] = fields["times_ns"][4]
        for trajectory in fields["trajectories"]:
            trajectory["point_times_ns"][5] = trajectory["point_times_ns"][4]
        self._reject(fields=fields)

    def test_sample_values_cannot_be_tampered(self):
        mutations = (
            ("x", 1, 1e-6),
            ("y", 2, 1e-6),
            ("theta", 3, 1e-6),
            ("v_i", 4, 1e-6),
            ("omega_i", 5, 1e-6),
            ("mode", 6, 99),
            ("C_i", 7, 1e-6),
        )
        for name, index, delta in mutations:
            fields = self._mutable_fields()
            fields["trajectories"][1]["points"][7][index] += delta
            with self.subTest(name=name):
                self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["formation_poses"][9][0] += 1e-6
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["formation_poses"][9][2] += 1e-6
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["stamp"] = [0, 1000000000]
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["frame_id"] = "odom"
        self._reject(fields=fields)

    def test_vehicle_metadata_cannot_be_tampered(self):
        for key, index, delta in (
                ("body_length_m", None, 1e-6),
                ("body_width_m", None, 1e-6)):
            fields = self._mutable_fields()
            fields["vehicles"][0][key] += delta
            with self.subTest(key=key):
                self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["vehicles"][1]["offset"][0] += 1e-6
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["vehicles"][1]["start_center"][0] += 1e-6
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["vehicles"][1]["start_center"][2] += 1e-6
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["vehicles"][1]["goal_center"][0] += 1e-6
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["vehicles"][1]["goal_center"][2] += 1e-6
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["vehicles"][0]["goal_heading_free"] = True
        self._reject(fields=fields)

    def test_envelope_identity_and_versions_cannot_be_tampered(self):
        self._reject(backend="uniform_v02")
        self._reject(semantic_version="virtual_rigid_a.playback.v0")
        self._reject(model_version="virtual_rigid_a.timed.v0")
        self._reject(certification="none")
        self._reject(plan_id=self.plan_id + 1)
        self._reject(formation_id="other_formation")
        self._reject(digest="0" * 64)
        self._reject(model_json=self.envelope["model_json"].replace(
            '"path"', '"path" ', 1))
        self._reject(model_json=self.envelope["model_json"][:-1])
        self._reject(fields=dict(self._mutable_fields(), plan_id=1))
        self._reject(fields=dict(
            self._mutable_fields(), formation_id="other_formation"))

    def test_envelope_key_set_is_exact(self):
        self._reject(drop=("certification",))
        self._reject(drop=("trajectory_fields",))
        extra = dict(self._mutable_fields())
        with self.assertRaises(VirtualAContractError):
            verify_envelope_fields({})
        envelope = dict(self.envelope)
        envelope["not_a_field"] = True
        with self.assertRaises(VirtualAContractError):
            verify_envelope_fields(envelope)

    def test_retained_request_is_required_and_must_match(self):
        self._reject(drop=("expected_request",), request=None)
        self._reject(request=g5.load_scene("S1")["request"])
        self._reject(request=dict(request_payload(
            g5.load_scene("S1")["request"])))

    def test_original_request_payload_is_accepted(self):
        verified = self._verify(request=request_payload(
            self.case["map_request"].request))
        self.assertEqual(verified.model_sha256, self.digest)

    def test_admitted_context_is_required_and_bound(self):
        self._reject(drop=("expected_context",), context=False)
        context = copy.deepcopy(self.envelope["expected_context"])
        context["stage"] = "G3"
        self._reject(context=context)
        context = copy.deepcopy(self.envelope["expected_context"])
        context["request_sha256"] = "0" * 64
        self._reject(context=context)
        context = copy.deepcopy(self.envelope["expected_context"])
        context["timing_config"]["safety_factor"] += 0.01
        self._reject(context=context)

    def test_grid_digest_is_bound(self):
        self._reject(expected_grid_sha256="0" * 64)

    def test_admission_record_is_bound_to_the_request(self):
        for key, value in (
                ("plan_id", self.plan_id + 1),
                ("formation_id", "other_formation"),
                ("request_sha256", "0" * 64),
                ("planning_context_sha256", "0" * 64),
                ("grid_sha256", "0" * 64),
                ("initial_context_id", "other"),
                ("goal_policy", "FREE")):
            admission = copy.deepcopy(self.envelope["expected_admission"])
            admission[key] = value
            with self.subTest(key=key):
                self._reject(admission=admission)

    def test_admission_initial_source_must_be_the_request_source(self):
        admission = copy.deepcopy(self.envelope["expected_admission"])
        admission["initial_source"] = "verified_terminal"
        self._reject(admission=admission)

    def test_non_authoritative_joint_arrays_are_rejected(self):
        fields = self._mutable_fields()
        fields["trajectories"][0]["points"].append(
            copy.deepcopy(fields["trajectories"][0]["points"][-1]))
        fields["trajectories"][0]["point_times_ns"].append(
            fields["trajectories"][0]["point_times_ns"][-1])
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["formation_poses"].pop()
        self._reject(fields=fields)
        fields = self._mutable_fields()
        fields["vehicles"].pop()
        self._reject(fields=fields)

    def test_f_only_envelope_rejects_an_unexpected_formation_binding(self):
        with self.assertRaises(VirtualAContractError) as caught:
            self._verify(expected_formation={"scope": "three_vehicle_union_v1"})
        self.assertEqual(caught.exception.reason,
                         "PLAYBACK_VERIFICATION_FAILED")
        self.assertIn("unexpected formation corridor binding",
                      str(caught.exception))


class UnionFormationEnvelopeTest(unittest.TestCase):
    """TASK03 Step 6: union plans carry the sealed formation binding.

    The case is a real pipeline run with the explicit
    ``three_vehicle_union_v1`` scope; the envelope is the same atomic
    transport as f_only.  The verifier must rebuild the retained template
    and chain, re-verify the continuous certificate against the published
    path, and reject a missing, stale or cross-wired binding.
    """

    @classmethod
    def setUpClass(cls):
        request = g4.make_request(
            FormationPose(0.0, 0.0, 0.0), FormationPose(6.0, 0.0, 0.0),
            plan_id=7101)
        grid = g4.empty_grid()
        planner = research_planner_config(
            guidance_mode="corridor", corridor_enforcement="hard",
            formation_corridor_scope=
                FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
            max_initials=3, max_seconds=20.0)
        cls.map_request = g4.map_request(request, grid, planner=planner)
        cls.result = solve(cls.map_request)
        cls.snapshot = g5.snapshot_from_request(request)
        cls.envelope = g5.envelope_for_result(
            result=cls.result, request=cls.map_request.request,
            context=cls.map_request.context(),
            grid_sha256=cls.map_request.grid_sha256,
            snapshot=cls.snapshot)

    def _binding(self):
        audit = self.result.formation_corridor_audit
        rounds = self.result.search["corridor"]["rounds"]
        return {
            "scope": FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
            "planning_context_sha256": self.result.planning_context_sha256,
            "template": copy.deepcopy(audit["template"]),
            "chain": copy.deepcopy(rounds[-1]["chain"]),
            "certificate": copy.deepcopy(audit["certificate"]),
        }

    def _verify(self, binding="default"):
        envelope = dict(self.envelope)
        if binding != "absent":
            envelope["expected_formation"] = (
                self._binding() if binding == "default" else binding)
        return verify_envelope_fields(envelope)

    def _reject(self, binding="default", text=None):
        with self.assertRaises(VirtualAContractError) as caught:
            self._verify(binding)
        self.assertEqual(caught.exception.reason,
                         "PLAYBACK_VERIFICATION_FAILED")
        if text is not None:
            self.assertIn(text, str(caught.exception))
        return caught.exception

    def test_union_plan_requires_and_accepts_the_retained_binding(self):
        self.assertEqual(self.result.status, "SUCCESS",
                         self.result.as_dict())
        self.assertEqual(self.result.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        certificate = self.result.formation_certificate
        self.assertEqual(certificate["status"], "VERIFIED")
        verified = self._verify()
        self.assertEqual(verified.model_sha256, self.result.model_sha256)

    def test_missing_binding_is_rejected_for_a_union_plan(self):
        self._reject("absent", "no retained formation corridor binding")

    def test_v2_template_payload_rebuilds_in_the_retained_binding(self):
        """The retained binding carries the sealed v2 F-region payload."""
        template_payload = self.result.formation_corridor_audit["template"]
        self.assertEqual(template_payload["schema_version"],
                         FORMATION_CORRIDOR_TEMPLATE_SCHEMA)
        self.assertEqual(template_payload["compiler_version"],
                         FORMATION_CORRIDOR_COMPILER_VERSION)
        self.assertTrue(template_payload["segments"])
        for segment in template_payload["segments"]:
            self.assertIn("f_region_index", segment)
            self.assertGreaterEqual(int(segment["f_region_index"]), 0)
        rebuilt = formation_template_from_payload(template_payload)
        self.assertEqual(rebuilt.as_dict(), template_payload)
        self.assertEqual(rebuilt.template_id,
                         self.result.formation_template_id)
        # The binding rebuild used by the playback verifier carries exactly
        # the five keys and the sealed template identity.
        binding = self._binding()
        self.assertEqual(set(binding), {
            "scope", "planning_context_sha256", "template", "chain",
            "certificate"})
        self.assertEqual(binding["template"]["template_id"],
                         rebuilt.template_id)

    def test_template_chain_and_certificate_are_bound(self):
        mutations = (
            ("template", lambda value: value.update(
                {"margin_m": value["margin_m"] + 1e-6})),
            ("chain", lambda value: value.update(
                {"route_source": value["route_source"] + "_stale"})),
            ("certificate", lambda value: value.update(
                {"max_outward_residual_m": -1.0})),
        )
        for key, mutate in mutations:
            binding = self._binding()
            mutate(binding[key])
            with self.subTest(key=key):
                self._reject(binding)

    def test_binding_context_must_match_the_admitted_context(self):
        binding = self._binding()
        binding["planning_context_sha256"] = "0" * 64
        self._reject(binding, "planning context differs")

    def test_binding_key_set_is_exact(self):
        binding = self._binding()
        binding["extra"] = True
        self._reject(binding, "formation corridor binding keys")


class FrameAndMarkerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        case = g5.solved("S2", "straight")
        cls.envelope = g5.envelope_for(case)
        cls.verified = verify_envelope_fields(cls.envelope)
        cls.trajectory = cls.verified.trajectory
        cls.plan = cls.verified.plan
        cls.request = case["map_request"].request
        cls.plan_id = cls.request.plan_id
        cls.formation_id = cls.request.formation_id
        cls.digest = cls.envelope["model_sha256"]

    def _status(self, **overrides):
        document = {
            "plan_id": self.plan_id,
            "formation_id": self.formation_id,
            "backend": "virtual_rigid_a",
            "model_sha256": self.digest,
            "status": "SUCCESS",
            "feasible": True,
            "executable": True,
            "trajectory_published": True,
        }
        document.update(overrides)
        return document

    def _times(self):
        duration = self.trajectory.duration
        values = [-1.0, 0.0, 0.001, 0.019, 0.020, 3.14159,
                  duration / 3.0, duration / 2.0,
                  duration - 1e-6, duration,
                  duration + 1e-6, duration + 100.0]
        for index in range(120):
            values.append(duration * (index + 0.37) / 120.0)
        return values

    def test_playback_frames_are_derived_from_the_model(self):
        for elapsed in self._times():
            clamped = min(max(elapsed, 0.0), self.trajectory.duration)
            state = self.plan.state_at(clamped)
            frame = frame_at(self.trajectory, elapsed)
            self.assertEqual(frame.frame_id, "map")
            self.assertEqual(frame.stamp, self.envelope["trajectory_fields"][
                "stamp"])
            self.assertEqual(frame.formation_x, state.qf.x)
            self.assertEqual(frame.formation_y, state.qf.y)
            self.assertEqual(frame.formation_psi, state.qf.psi)
            self.assertEqual(frame.vehicle_ids, (1, 2, 3))
            for vehicle_id in frame.vehicle_ids:
                expected = state.vehicle(vehicle_id)
                actual = frame.vehicle_map()[vehicle_id]
                self.assertEqual(actual.vehicle_id, vehicle_id)
                self.assertEqual(actual.center_x, expected.x)
                self.assertEqual(actual.center_y, expected.y)
                self.assertEqual(actual.theta, expected.theta)
                self.assertEqual(actual.mode, ACKERMANN)
                self.assertEqual(actual.mode_control, expected.C_i)
                self.assertEqual(actual.center_speed_mps, expected.v_i)
            if elapsed <= 0.0:
                self.assertEqual(frame.state, HOLD_START)
            elif elapsed >= self.trajectory.duration:
                self.assertEqual(frame.state, HOLD_END)
                self.assertEqual(frame.clamped_elapsed_s,
                                 self.trajectory.duration)
            else:
                self.assertEqual(frame.state, PLAYING)
                self.assertEqual(frame.clamped_elapsed_s, elapsed)

    def test_rigidity_holds_at_every_interpolated_frame(self):
        expected_distances = {
            (1, 2): math.sqrt(2.0),
            (1, 3): math.sqrt(2.0),
            (2, 3): 2.0,
        }
        for elapsed in self._times():
            frame = frame_at(self.trajectory, elapsed)
            states = frame.vehicle_map()
            for (first, second), distance in expected_distances.items():
                actual = math.hypot(
                    states[first].center_x - states[second].center_x,
                    states[first].center_y - states[second].center_y)
                self.assertLessEqual(abs(actual - distance), 1e-6)

    def test_terminal_hold_matches_the_last_sample(self):
        frame = frame_at(self.trajectory, self.plan.duration_s + 1.0)
        self.assertTrue(frame.terminal)
        center_error, heading_error = terminal_errors(
            self.trajectory, frame)
        self.assertLessEqual(center_error, 1e-6)
        self.assertLessEqual(heading_error, 1e-6)

    def test_markers_follow_the_authoritative_frame(self):
        frame = frame_at(self.trajectory, self.plan.duration_s / 2.0)
        markers, namespaces, audit_state, executable = build_markers(
            self.trajectory, frame, status=self._status())
        self.assertTrue(executable)
        self.assertEqual(audit_state, "FEASIBLE")
        self.assertEqual(len(markers), 17)
        self.assertEqual(len({marker.marker_id for marker in markers}),
                         len(markers))
        self.assertEqual(tuple(sorted(namespaces)), namespaces)
        for vehicle_id in (1, 2, 3):
            self.assertIn(vehicle_namespace(
                self.formation_id, vehicle_id), namespaces)
            vehicle_markers = [marker for marker in markers
                               if marker.namespace == vehicle_namespace(
                                   self.formation_id, vehicle_id)]
            self.assertEqual(len(vehicle_markers), 4)
            for marker in vehicle_markers:
                self.assertEqual(marker.frame_id, "map")
            arrow = next(marker for marker in vehicle_markers
                         if marker.marker_type == "ARROW")
            state = frame.vehicle_map()[vehicle_id]
            start, end = arrow.points
            self.assertAlmostEqual(start[0], state.center_x, places=12)
            self.assertAlmostEqual(start[1], state.center_y, places=12)
            angle = math.atan2(end[1] - start[1], end[0] - start[0])
            self.assertLess(abs(math.atan2(
                math.sin(angle - state.theta),
                math.cos(angle - state.theta))), 1e-12)
        links = [marker for marker in markers
                 if marker.marker_type == "LINE_LIST"]
        self.assertEqual(len(links), 1)
        self.assertEqual(len(links[0].points), 6)
        self.assertTrue(all(
            math.isfinite(value) for point in links[0].points
            for value in point))

    def test_status_document_matches_the_verified_envelope(self):
        frame = frame_at(self.trajectory, self.plan.duration_s / 2.0)
        markers, namespaces, _, _ = build_markers(
            self.trajectory, frame, status=self._status())
        document = status_document(
            self.trajectory, frame, self._status(),
            marker_count=len(markers), namespaces=namespaces,
            backend="virtual_rigid_a",
            semantic_version="virtual_rigid_a.playback.v1")
        self.assertEqual(document["schema"], "formation_playback/v1")
        self.assertEqual(document["state"], PLAYING)
        self.assertEqual(document["backend"], "virtual_rigid_a")
        self.assertEqual(document["model_sha256"], self.digest)
        self.assertEqual(document["semantic_version"],
                         "virtual_rigid_a.playback.v1")
        self.assertEqual(document["plan_id"], self.plan_id)
        self.assertEqual(document["formation_id"], self.formation_id)
        self.assertTrue(document["executable"])
        self.assertFalse(document["geometry_only"])
        self.assertEqual(document["certification"], "FEASIBLE")
        self.assertEqual(document["audit_status"], "FEASIBLE")
        self.assertEqual(document["marker_count"], len(markers))
        self.assertEqual(document["marker_namespaces"], list(namespaces))
        self.assertEqual(document["sample_count"],
                         self.trajectory.sample_count)
        self.assertEqual(document["terminal"], False)
        for actual, expected in zip(document["vehicles"],
                                    frame.vehicles):
            self.assertEqual(actual["vehicle_id"], expected.vehicle_id)
            self.assertEqual(actual["center"],
                             [expected.center_x, expected.center_y])
            self.assertEqual(actual["theta"], expected.theta)
            self.assertEqual(actual["mode"], ACKERMANN)
            self.assertEqual(actual["mode_control"],
                             expected.mode_control)

    def test_status_for_another_identity_never_certifies_the_display(self):
        frame = frame_at(self.trajectory, self.plan.duration_s / 2.0)
        variants = (
            self._status(model_sha256="f" * 64),
            self._status(backend="uniform_v02"),
            self._status(formation_id="other"),
            self._status(plan_id=self.plan_id + 1),
        )
        for status in variants:
            with self.subTest(status=status):
                document = status_document(
                    self.trajectory, frame, status)
                self.assertFalse(document["executable"])
                self.assertTrue(document["geometry_only"])
                self.assertEqual(document["certification"],
                                 "GEOMETRY_ONLY")
                self.assertEqual(document["audit_status"],
                                 "GEOMETRY_ONLY")
        legacy = status_document(self.trajectory, frame, None)
        self.assertFalse(legacy["executable"])
        self.assertTrue(legacy["geometry_only"])


class NoMotionEnvelopeTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.grid = g4.empty_grid()
        cls.request, _, _ = g4.retime(g4.hold_path(), cls.grid)
        cls.map_request = g4.map_request(cls.request, cls.grid)
        cls.result = plan_virtual_a(cls.map_request)
        assert cls.result.status == "NO_MOTION"
        cls.snapshot = g5.snapshot_from_request(cls.request)
        cls.envelope = g5.envelope_for_result(
            result=cls.result, request=cls.request,
            context=cls.map_request.context(),
            grid_sha256=cls.map_request.grid_sha256,
            snapshot=cls.snapshot)
        cls.verified = verify_envelope_fields(cls.envelope)
        cls.trajectory = cls.verified.trajectory
        cls.plan = cls.verified.plan
        cls.plan_id = cls.request.plan_id
        cls.formation_id = cls.request.formation_id
        cls.fields = cls.envelope["trajectory_fields"]

    def test_no_motion_hold_has_one_frame_at_zero(self):
        self.assertEqual(self.plan.kind, "hold")
        self.assertEqual(self.plan.duration_s, 0.0)
        self.assertEqual(self.plan.total_S_m, 0.0)
        self.assertEqual(tuple(self.fields["times_ns"]), (0,))
        self.assertEqual(self.trajectory.times_from_start, (0.0,))
        self.assertEqual(self.trajectory.sample_count, 1)
        frame = frame_at(self.trajectory, 0.0)
        self.assertEqual(frame.state, HOLD_START)
        self.assertEqual(frame.formation_x, self.plan.state_at(0.0).qf.x)
        self.assertEqual(frame.formation_y, self.plan.state_at(0.0).qf.y)
        for vehicle in frame.vehicles:
            self.assertEqual(vehicle.center_speed_mps, 0.0)
            self.assertEqual(vehicle.mode, ACKERMANN)

    def test_no_motion_rejects_a_second_sample(self):
        envelope = dict(self.envelope)
        fields = json.loads(json.dumps(self.envelope["trajectory_fields"]))
        for trajectory in fields["trajectories"]:
            trajectory["points"].append(
                copy.deepcopy(trajectory["points"][0]))
            trajectory["point_times_ns"].append(0)
        fields["formation_poses"].append(fields["formation_poses"][0])
        fields["times_from_start"].append(0.0)
        fields["times_ns"].append(0)
        envelope["trajectory_fields"] = fields
        with self.assertRaises(VirtualAContractError):
            verify_envelope_fields(envelope)

    def test_no_motion_marker_frame_is_authoritative(self):
        frame = frame_at(self.trajectory, 0.0)
        markers, namespaces, audit_state, executable = build_markers(
            self.trajectory, frame)
        self.assertEqual(len(markers), 17)
        self.assertIsInstance(audit_state, str)
        self.assertIsInstance(executable, bool)
        for vehicle in frame.vehicles:
            namespace = vehicle_namespace(
                self.formation_id, vehicle.vehicle_id)
            self.assertIn(namespace, namespaces)


class RawRequestReconstructionTest(unittest.TestCase):
    """The receiver rebuilds the request from raw ROS fields only."""

    @classmethod
    def setUpClass(cls):
        cls.free = g5.solved("S2", "free")["map_request"].request
        cls.straight = g5.solved("S2", "straight")["map_request"].request

    @staticmethod
    def _document(request, *, free=None, order=None):
        free = (request.goal_policy == GoalPolicy.FREE
                if free is None else free)
        centers = request.model.centers(request.start_qf)
        goals = request.model.centers(request.goal_qf)
        rows = []
        for offset in request.model.offsets:
            shape = request.vehicle_shapes[offset.vehicle_id]
            rows.append({
                "vehicle_id": offset.vehicle_id,
                "offset_x": offset.x,
                "offset_y": offset.y,
                "start_x": centers[offset.vehicle_id][0],
                "start_y": centers[offset.vehicle_id][1],
                "start_theta": request.start_headings[offset.vehicle_id],
                "goal_x": goals[offset.vehicle_id][0],
                "goal_y": goals[offset.vehicle_id][1],
                "goal_theta": request.goal_qf.psi,
                "goal_heading_free": free,
                "body_length_m": shape.length_m,
                "body_width_m": shape.width_m,
            })
        if order is not None:
            rows = [rows[index] for index in order]
        return {
            "plan_id": request.plan_id,
            "formation_id": request.formation_id,
            "formation_start": {
                "x": request.start_qf.x,
                "y": request.start_qf.y,
                "theta": request.start_qf.psi,
            },
            "formation_goal": {
                "x": request.goal_qf.x,
                "y": request.goal_qf.y,
                "theta": request.goal_qf.psi,
            },
            "allow_reverse": False,
            "vehicles": rows,
        }

    def _rebuild(self, request, **kwargs):
        return virtual_a_request_from_raw_request(
            self._document(request, **kwargs),
            initial_source=request.initial.source,
            initial_context_id=request.initial.context_id)

    def test_free_raw_request_matches_the_admitted_fingerprint(self):
        rebuilt = self._rebuild(self.free)
        self.assertEqual(request_fingerprint(rebuilt),
                         request_fingerprint(self.free))
        self.assertEqual(rebuilt.goal_policy, GoalPolicy.FREE)
        self.assertEqual(rebuilt.initial.source, self.free.initial.source)
        self.assertEqual(rebuilt.initial.context_id,
                         self.free.initial.context_id)

    def test_straight_raw_request_matches_the_admitted_fingerprint(self):
        rebuilt = self._rebuild(self.straight)
        self.assertEqual(request_fingerprint(rebuilt),
                         request_fingerprint(self.straight))
        self.assertEqual(rebuilt.goal_policy, GoalPolicy.STRAIGHT)

    def test_vehicle_message_order_does_not_change_the_request(self):
        rebuilt = self._rebuild(self.free, order=[2, 0, 1])
        self.assertEqual(request_fingerprint(rebuilt),
                         request_fingerprint(self.free))

    def test_mixed_goal_heading_flags_are_rejected(self):
        document = self._document(self.free)
        document["vehicles"][0]["goal_heading_free"] = False
        with self.assertRaises(PlaybackValidationError):
            virtual_a_request_from_raw_request(
                document, initial_source="rviz_reset",
                initial_context_id="virtual-a-mixed")

    def test_non_rigid_start_centre_is_rejected(self):
        document = self._document(self.free)
        document["vehicles"][0]["start_x"] += 1e-3
        with self.assertRaises(PlaybackValidationError):
            virtual_a_request_from_raw_request(
                document, initial_source="rviz_reset",
                initial_context_id="virtual-a-nonrigid")

    def test_straight_goal_heading_must_equal_the_goal_psi(self):
        document = self._document(self.straight)
        document["vehicles"][0]["goal_theta"] += 0.1
        with self.assertRaises(PlaybackValidationError):
            virtual_a_request_from_raw_request(
                document, initial_source="verified_terminal",
                initial_context_id="virtual-a-heading")

    def test_missing_raw_field_is_rejected(self):
        document = self._document(self.free)
        del document["vehicles"][0]["body_width_m"]
        with self.assertRaises(PlaybackValidationError):
            virtual_a_request_from_raw_request(
                document, initial_source="rviz_reset",
                initial_context_id="virtual-a-missing")


class InvalidPlaybackTest(unittest.TestCase):

    def test_frame_requires_a_verified_trajectory(self):
        with self.assertRaises(PlaybackValidationError):
            frame_at(object(), 0.0)

    def test_status_requires_a_trajectory_and_frame(self):
        with self.assertRaises(PlaybackValidationError):
            status_document(None, None, None)

    def test_backend_dispatch_rejects_unknown_backends(self):
        from fwis_v3_optimizer.formation_playback import (
            verify_playback_plan)
        with self.assertRaises(PlaybackValidationError):
            verify_playback_plan(
                backend="not_a_backend", plan_id=1,
                formation_id="x", semantic_version="x",
                model_version="x", model_sha256="0" * 64,
                model_json="{}", certification="x",
                trajectory_fields={})


if __name__ == "__main__":
    unittest.main()
