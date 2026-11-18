"""V0 fixed-corner HUD, compact labels and audit-column tests.

These tests are ROS-independent.  They pin the text seen in the OverlayText
panel, the concise body labels and the identity gate that prevents a stale
quality report from painting PASS columns onto a newer playback.
"""
from pathlib import Path
import math
import re
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(ROOT / "tests"))

from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    PLAYBACK_PLAN_BACKEND,
    VIRTUAL_PLAYBACK_BACKEND,
    PlaybackMarkerOptions,
    PlaybackValidationError,
    build_hold_markers,
    build_markers,
    frame_at,
    hud_text,
    quality_identity_matches,
    status_document,
    vehicle_namespace,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    COLLISION_SCOPE,
    DYNAMICS_SCOPE,
    GEOMETRY_SCOPE,
)
from test_formation_playback import (  # noqa: E402
    fields,
    point,
    trajectory,
)
from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    ACKERMANN,
    OBLIQUE,
    PlaybackVehicleSpec,
)

DIGEST = "a" * 64
CONTEXT = "c" * 64
REQUEST = "e" * 64


def quality_document(*, plan_id=7, formation_id="wedge_3",
                     backend=VIRTUAL_PLAYBACK_BACKEND,
                     model_sha256=DIGEST, request_sha256=REQUEST,
                     planning_context_sha256=CONTEXT,
                     audits=None):
    if audits is None:
        audits = {
            "geometry": {"scope": GEOMETRY_SCOPE, "passed": True},
            "dynamics": {"scope": DYNAMICS_SCOPE, "passed": True},
            "collision": {"scope": COLLISION_SCOPE, "passed": True},
        }
    return {
        "schema": "formation_quality_report/v1",
        "plan_id": plan_id,
        "formation_id": formation_id,
        "backend": backend,
        "model_sha256": model_sha256,
        "request_sha256": request_sha256,
        "planning_context_sha256": planning_context_sha256,
        "virtual_result": {"audits": audits},
        "pipeline": {"selection": {"selected_candidate_id": 1}},
    }


def playback_document(*, plan_id=7, formation_id="wedge_3",
                      backend=VIRTUAL_PLAYBACK_BACKEND,
                      model_sha256=DIGEST, request_sha256=REQUEST,
                      planning_context_sha256=CONTEXT):
    model = trajectory(plan_id=plan_id, formation_id=formation_id)
    frame = frame_at(model, 0.5)
    return status_document(
        model, frame, None,
        model_sha256=model_sha256,
        backend=backend,
        request_sha256=request_sha256,
        planning_context_sha256=planning_context_sha256)


def audit_line(text):
    return next(
        line for line in text.splitlines()
        if line.startswith("AUDIT geometry="))


def column(text, name):
    match = re.search(r"{}[=]([A-Z]+)".format(name), audit_line(text))
    if match is None:
        # collision is on the continuation line
        match = re.search(r"collision=([A-Z]+)", text)
    if match is None:
        raise AssertionError("no {} column in {!r}".format(name, text))
    return match.group(1)


class CompactVehicleLabelTest(unittest.TestCase):

    def test_label_uses_frame_id_mode_and_centre_speed(self):
        samples = {
            0: (point(0.0, 0.0, 0.0, 0.0, ACKERMANN, 0.0, 1.234),
                point(1.0, 1.0, 0.0, 0.0, ACKERMANN, 0.0, 1.234),
                point(2.0, 2.0, 0.0, 0.0, ACKERMANN, 0.0, 1.234)),
            1: (point(0.0, 0.0, 1.2, math.pi / 2.0, OBLIQUE,
                      math.pi / 4.0, 0.8),
                point(1.0, 1.0, 1.2, math.pi / 2.0, OBLIQUE,
                      math.pi / 4.0, 0.8),
                point(2.0, 2.0, 1.2, math.pi / 2.0, OBLIQUE,
                      math.pi / 4.0, 0.8)),
        }
        model = trajectory(samples=samples)
        frame = frame_at(model, 0.5)
        markers, _, _, _ = build_markers(model, frame)
        for vehicle_id, mode, speed in ((0, "A", 1.234), (1, "O", 0.8)):
            state = frame.vehicle_map()[vehicle_id]
            namespace = vehicle_namespace("wedge_3", vehicle_id)
            label = next(
                marker for marker in markers
                if marker.namespace == namespace
                and marker.marker_type == "TEXT_VIEW_FACING")
            match = re.match(
                r"^car{} \| {} \| v=([0-9.]+) m/s$".format(
                    vehicle_id, mode), label.text)
            self.assertIsNotNone(match, label.text)
            self.assertLessEqual(
                abs(float(match.group(1)) - speed), 0.005)
            self.assertLessEqual(
                abs(label.position[0] - state.center_x), 1e-9)
            self.assertLessEqual(
                abs(label.position[1] - state.center_y), 1e-9)
            self.assertNotIn("ω", label.text)

    def test_static_hold_labels_report_zero_speed_without_a_model(self):
        vehicles = tuple(
            (PlaybackVehicleSpec(vehicle_id, a, b, 0.738, 0.5),
             0.0 + a, b, 0.0)
            for vehicle_id, (a, b) in zip(
                (1, 2, 3), ((1.0, 0.0), (0.0, 1.0), (0.0, -1.0))))
        markers = build_hold_markers(
            "cargo_wedge", (0.0, 0.0, 0.0), vehicles,
            "GOAL_FORMATION_COLLISION")
        labels = [marker.text for marker in markers
                  if marker.marker_type == "TEXT_VIEW_FACING"
                  and marker.text.startswith("car")]
        self.assertEqual(labels, [
            "car1 | STATIC | v=0.00 m/s",
            "car2 | STATIC | v=0.00 m/s",
            "car3 | STATIC | v=0.00 m/s",
        ])
        document = {
            "state": "STATIC", "plan_id": 4,
            "formation_id": "cargo_wedge",
            "backend": VIRTUAL_PLAYBACK_BACKEND,
            "model_sha256": None,
            "vehicles": [{"vehicle_id": 1}, {"vehicle_id": 2},
                         {"vehicle_id": 3}],
        }
        text = hud_text(document, "HOLD_END", None)
        self.assertIn("car1 | STATIC | v=0.00 m/s", text)
        self.assertIn("car2 | STATIC | v=0.00 m/s", text)
        self.assertIn("car3 | STATIC | v=0.00 m/s", text)
        self.assertIn("Cargo", text)
        self.assertIn("STATIC / no executable trajectory", text)
        self.assertIn("AUDIT geometry=UNKNOWN", text)

    def test_hud_repeats_the_cargo_hold_text_and_reason_switch(self):
        document = {
            "state": "STATIC", "plan_id": 4,
            "formation_id": "cargo_wedge",
            "backend": VIRTUAL_PLAYBACK_BACKEND,
            "model_sha256": None,
            "reason": "PLANNING",
            "vehicles": [{"vehicle_id": 1}],
        }
        shown = hud_text(document, "PLANNING", None)
        self.assertIn("Cargo | PLANNING", shown)
        self.assertIn("STATIC / no executable trajectory", shown)
        compact = hud_text(document, "PLANNING", None, False)
        self.assertIn("\nCargo\n", compact)
        self.assertNotIn("Cargo | PLANNING", compact)
        self.assertIn("STATIC / no executable trajectory", compact)
        playing = playback_document()
        self.assertNotIn("Cargo", hud_text(playing, "PLAYING", None))
        with self.assertRaises(PlaybackValidationError):
            hud_text(document, "PLANNING", None, "no")

    def test_f_reference_label_is_retained(self):
        model = trajectory()
        frame = frame_at(model, 0.5)
        markers, _, _, _ = build_markers(model, frame)
        texts = [
            marker.text for marker in markers
            if marker.namespace == "formation/wedge_3/reference"
            and marker.marker_type == "TEXT_VIEW_FACING"]
        self.assertEqual(texts, ["F"])


class HUDTextTest(unittest.TestCase):

    def test_hud_reports_plan_formation_job_and_each_frame_vehicle(self):
        document = playback_document()
        text = hud_text(document, "PLAYING", None)
        self.assertIn("FORMATION HUD | plan 7", text)
        self.assertIn("formation wedge_3 | virtual_rigid_a", text)
        self.assertIn("job PLAYING | playback PLAYING", text)
        self.assertIn("car0 | A | v=1.00 m/s", text)
        self.assertIn("car1 | O | v=0.80 m/s", text)
        self.assertIn("AUDIT geometry=PENDING dynamics=PENDING", text)
        self.assertIn("collision=PENDING", text)
        self.assertNotIn("FEASIBLE", text)
        self.assertNotIn("ω", text)

    def test_matching_three_layer_quality_fills_all_columns(self):
        document = playback_document()
        text = hud_text(document, "PLAYING", quality_document())
        self.assertEqual(column(text, "geometry"), "PASS")
        self.assertEqual(column(text, "dynamics"), "PASS")
        self.assertEqual(column(text, "collision"), "PASS")

    def test_old_or_wrong_identity_quality_never_shows_pass(self):
        document = playback_document()
        variants = (
            quality_document(plan_id=8),
            quality_document(model_sha256="b" * 64),
            quality_document(formation_id="other"),
            quality_document(backend=PLAYBACK_PLAN_BACKEND),
            quality_document(planning_context_sha256="d" * 64),
            quality_document(request_sha256="f" * 64),
        )
        for quality in variants:
            text = hud_text(document, "PLAYING", quality)
            self.assertEqual(column(text, "geometry"), "UNKNOWN")
            self.assertEqual(column(text, "dynamics"), "UNKNOWN")
            self.assertEqual(column(text, "collision"), "UNKNOWN")

    def test_quality_requires_the_virtual_request_and_context(self):
        deliverable = playback_document()
        quality = quality_document()
        self.assertTrue(quality_identity_matches(deliverable, quality))
        without_context = dict(deliverable, planning_context_sha256=None)
        self.assertFalse(
            quality_identity_matches(without_context, quality))
        without_request = dict(deliverable, request_sha256=None)
        self.assertFalse(
            quality_identity_matches(without_request, quality))

    def test_failed_layer_and_wrong_scope_are_not_pass(self):
        quality = quality_document(audits={
            "geometry": {"scope": DYNAMICS_SCOPE, "passed": True},
            "dynamics": {"scope": DYNAMICS_SCOPE, "passed": False},
            "collision": {"scope": COLLISION_SCOPE},
        })
        text = hud_text(
            playback_document(), "PLAYING", quality)
        self.assertEqual(column(text, "geometry"), "UNKNOWN")
        self.assertEqual(column(text, "dynamics"), "FAIL")
        self.assertEqual(column(text, "collision"), "PENDING")

    def test_planning_without_a_quality_report_is_pending_not_pass(self):
        text = hud_text(playback_document(), "PLANNING", None)
        self.assertEqual(column(text, "geometry"), "PENDING")
        self.assertEqual(column(text, "dynamics"), "PENDING")
        self.assertEqual(column(text, "collision"), "PENDING")

    def test_failed_job_without_quality_is_unknown(self):
        text = hud_text(playback_document(), "FAILED", None)
        self.assertEqual(column(text, "geometry"), "UNKNOWN")
        self.assertEqual(column(text, "dynamics"), "UNKNOWN")
        self.assertEqual(column(text, "collision"), "UNKNOWN")

    def test_status_document_carries_the_identity_needed_by_the_hud(self):
        document = playback_document()
        self.assertEqual(document["request_sha256"], REQUEST)
        self.assertEqual(document["planning_context_sha256"], CONTEXT)
        self.assertEqual(document["model_sha256"], DIGEST)
        self.assertEqual(document["backend"], VIRTUAL_PLAYBACK_BACKEND)


class DisplaySwitchTest(unittest.TestCase):

    def test_status_text_switch_only_removes_the_long_map_block(self):
        model = trajectory()
        frame = frame_at(model, 0.5)
        full, full_namespaces, _, _ = build_markers(model, frame)
        compact, compact_namespaces, _, _ = build_markers(
            model, frame,
            PlaybackMarkerOptions(show_status_text=False))
        self.assertEqual(len(full), len(compact) + 1)
        self.assertIn("formation/wedge_3/status", full_namespaces)
        self.assertNotIn("formation/wedge_3/status", compact_namespaces)
        self.assertTrue(any(
            marker.text == "F" for marker in compact
            if marker.marker_type == "TEXT_VIEW_FACING"))

    def test_non_bool_status_switch_fails_closed(self):
        with self.assertRaises(PlaybackValidationError):
            PlaybackMarkerOptions(show_status_text="yes")

    def test_cargo_reason_switch_preserves_the_static_warning(self):
        vehicles = tuple(
            (PlaybackVehicleSpec(vehicle_id, a, b, 0.738, 0.5),
             0.0 + a, b, 0.0)
            for vehicle_id, (a, b) in zip(
                (1, 2, 3), ((1.0, 0.0), (0.0, 1.0), (0.0, -1.0))))
        full = build_hold_markers(
            "cargo_wedge", (0.0, 0.0, 0.0), vehicles,
            "GOAL_FORMATION_COLLISION")
        compact = build_hold_markers(
            "cargo_wedge", (0.0, 0.0, 0.0), vehicles,
            "GOAL_FORMATION_COLLISION", show_cargo_reason=False)
        full_text = next(marker.text for marker in full
                         if marker.text.startswith("Cargo"))
        compact_text = next(marker.text for marker in compact
                            if marker.text.startswith("Cargo"))
        self.assertIn("GOAL_FORMATION_COLLISION", full_text)
        self.assertNotIn("GOAL_FORMATION_COLLISION", compact_text)
        self.assertIn("STATIC / no executable trajectory", full_text)
        self.assertIn("STATIC / no executable trajectory", compact_text)

    def test_non_bool_cargo_switch_fails_closed(self):
        with self.assertRaises(PlaybackValidationError):
            build_hold_markers(
                "f", (0.0, 0.0, 0.0), (), "reason",
                show_cargo_reason="no")


if __name__ == "__main__":
    unittest.main()