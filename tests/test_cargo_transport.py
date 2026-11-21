"""Transport footprint, independent body headings and the legacy lock."""
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))
sys.path.insert(0, str(ROOT / "tools"))
from transport_footprint import footprint_metrics  # noqa: E402
from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_kinematics import (  # noqa: E402
    ACKERMANN, FormationKinematicBlock, KinematicLimits,
    audit_mode_assignment, enumerate_mode_assignments)
from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    PlaybackVehicleSpec, build_hold_markers)
from fwis_v3_optimizer.transport_contract import (  # noqa: E402
    contract_from_mapping)

CONFIG = (ROOT / "catkin_ws/src/fwis_v3_demo/config/"
          "transport_formation.yaml")
CONTRACT = contract_from_mapping(
    yaml.safe_load(CONFIG.read_text(encoding="utf-8")))
OFFSETS = CONTRACT.offsets


def offset_objects():
    return tuple(VehicleOffset(i + 1, *offset)
                 for i, offset in enumerate(OFFSETS))


class TransportFootprintTest(unittest.TestCase):

    def test_physical_footprint_of_the_authoritative_wedge(self):
        metrics = footprint_metrics(CONFIG)
        self.assertAlmostEqual(metrics["single_body_area_m2"], 0.369)
        self.assertAlmostEqual(metrics["three_bodies_union_area_m2"], 1.107)
        self.assertAlmostEqual(metrics["common_heading_hull_area_m2"], 3.345)
        self.assertEqual(metrics["common_heading_aabb_m"], [1.738, 2.5])
        self.assertAlmostEqual(metrics["common_heading_aabb_area_m2"], 4.345)
        self.assertAlmostEqual(metrics["body_circumradius_m"],
                               0.44571403388271275)
        self.assertAlmostEqual(metrics["heading_free_outer_radius_m"],
                               1.4457140338827128)
        self.assertAlmostEqual(metrics["heading_free_envelope_m2"],
                               6.5662084606402)
        self.assertAlmostEqual(metrics["required_axis_scale_factor"], 5.0)
        self.assertAlmostEqual(metrics["conservative_disk_scale_factor"],
                               5.782856135530851)
        self.assertGreater(metrics["conservative_disk_scale_factor"],
                           metrics["required_axis_scale_factor"])
        self.assertTrue(metrics["bodies_pairwise_separated"])

    def test_offsets_come_from_the_config_not_a_local_copy(self):
        metrics = footprint_metrics(CONFIG)
        self.assertEqual(metrics["vehicle_ids"], [1, 2, 3])
        self.assertEqual(metrics["offsets_m"],
                         [[1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])


class IndependentBodyHeadingTest(unittest.TestCase):

    def rotation_block(self, dpsi):
        return FormationKinematicBlock.from_primitive(
            FormationPose(5.0, 5.0, 0.0),
            SimpleNamespace(geometry_type=1, dx=0.0, dy=0.0, dpsi=dpsi,
                            pivot_x=0.0, pivot_y=0.0),
            offset_objects())

    def test_distinct_tangent_headings_are_a_legal_rotation(self):
        block = self.rotation_block(0.5)
        headings = {1: math.pi / 2.0, 2: math.pi, 3: 0.0}
        limits = KinematicLimits()  # require_cargo_heading is False by default
        assigned = enumerate_mode_assignments(
            block, headings, limits, allow_reverse=True)[0]
        self.assertTrue(assigned.success, assigned.detail)
        self.assertTrue(audit_mode_assignment(
            assigned, limits, allow_reverse=True).success)
        self.assertEqual({segment.mode for segment in assigned.segments},
                         {ACKERMANN})
        # The three body headings are pairwise different; theta_i is not a
        # copy of the cargo heading.
        wrapped = {round(math.remainder(angle, 2.0 * math.pi), 9)
                   for angle in headings.values()}
        self.assertEqual(len(wrapped), 3)

    def test_a_non_tangent_heading_is_not_silently_accepted(self):
        block = self.rotation_block(0.5)
        headings = {1: 0.0, 2: math.pi, 3: 0.0}
        rejected = enumerate_mode_assignments(
            block, headings, KinematicLimits(), allow_reverse=True)[0]
        self.assertFalse(rejected.success)
        self.assertEqual(rejected.reason, "HEADING_BOUNDARY_MISMATCH")


class LegacyCargoHeadingLockTest(unittest.TestCase):
    """The old theta_i=psi_F lock is an explicit restricted experiment."""

    def translation_block(self):
        return FormationKinematicBlock.from_primitive(
            FormationPose(4.0, 4.0, 0.31),
            SimpleNamespace(geometry_type=0, dx=1.0, dy=0.4, dpsi=0.0,
                            pivot_x=0.0, pivot_y=0.0),
            offset_objects())

    def test_equal_headings_pass_under_the_legacy_lock(self):
        block = self.translation_block()
        limits = KinematicLimits(require_cargo_heading=True)
        headings = {v.vehicle_id: 0.31 for v in offset_objects()}
        assigned = enumerate_mode_assignments(block, headings, limits,
                                              allow_reverse=True)[0]
        self.assertTrue(assigned.success, assigned.detail)
        self.assertTrue(audit_mode_assignment(
            assigned, limits, allow_reverse=True).success)
        headings[2] += 0.1
        rejected = enumerate_mode_assignments(block, headings, limits)[0]
        self.assertFalse(rejected.success)
        self.assertEqual(rejected.reason, "CARGO_HEADING_MISMATCH")

    def test_legacy_rotation_still_reports_the_old_missing_model(self):
        block = FormationKinematicBlock.from_primitive(
            FormationPose(4.0, 4.0, 0.31),
            SimpleNamespace(geometry_type=1, dx=0.0, dy=0.0, dpsi=0.4,
                            pivot_x=0.0, pivot_y=0.0),
            offset_objects())
        result = enumerate_mode_assignments(
            block, {1: 0.31, 2: 0.31, 3: 0.31},
            KinematicLimits(require_cargo_heading=True),
            allow_reverse=True)[0]
        self.assertFalse(result.success)
        self.assertEqual(result.reason,
                         "CARGO_ROTATION_REQUIRES_GENERAL_4WIS")


class StaticPreviewTest(unittest.TestCase):

    def test_static_preview_has_three_bodies_and_no_path(self):
        vehicles = tuple((PlaybackVehicleSpec(i + 1, a, b, .738, .5),
                          a, b, 0.0) for i, (a, b) in enumerate(OFFSETS))
        markers = build_hold_markers("cargo_wedge", (0, 0, 0), vehicles,
                                     "GOAL_FORMATION_COLLISION")
        bodies = [m for m in markers if m.marker_type == "CUBE"]
        self.assertEqual(len(bodies), 3)
        self.assertFalse(any(m.marker_type == "LINE_STRIP" for m in markers))
        for body, (_spec, x, y, theta) in zip(bodies, vehicles):
            self.assertEqual(body.scale, (0.738, 0.500, 0.338))
            self.assertEqual(body.position[:2], (x, y))
            self.assertEqual(body.yaw, theta)
            self.assertEqual(body.color[3], 1.0)
        self.assertEqual(len({m.marker_id for m in markers}), len(markers))
        self.assertTrue(any("STATIC" in m.text for m in markers))
        self.assertTrue(any("GOAL_FORMATION_COLLISION" in m.text
                            for m in markers))


class MapScaleEntryTest(unittest.TestCase):

    def test_map_default_and_explicit_override(self):
        for args, scale in ((["--map", "map02"], "2.56"),
                            (["--map", "map04"], "1.8"),
                            (["--map", "map08"], "3.5"),
                            ([], "2.6"),
                            (["--map", "map16", "--scale", "2.0"], "2.0")):
            result = subprocess.run(
                ["bash", str(ROOT / "run_demo.sh"), "--dry-run", *args],
                cwd=ROOT, capture_output=True, text=True, check=True)
            self.assertIn("formation_demo.launch", result.stdout)
            self.assertIn(" scale=" + scale + " ", result.stdout)
            self.assertNotIn("planning_backend:=", result.stdout)


if __name__ == "__main__":
    unittest.main()
