"""Regressions for the seed-preserving continuous A guidance mode."""
import math
import time
import unittest
from dataclasses import replace

import numpy as np

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a.audit import audit_geometry
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    FOOTPRINT_GUIDANCE, VirtualAContractError, research_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.footprint_seed import (
    _chord_swept_box, _corners, _cover_passes, _refinement_limits,
    _world_corners, build_footprint_regions,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import VirtualAPath
from fwis_v3_optimizer.formation_continuous_a.pipeline import plan_virtual_a

from virtual_a_g4_support import (
    MODEL, empty_grid, make_request, map_request, ramp_path, straight_path,
    straight_profile,
)


def narrow_corridor():
    grid = Grid(np.zeros((80, 160), dtype=bool), 0.05, -2.0, -2.0)
    grid.occ[:12] = True
    grid.occ[68:] = True
    return grid


class FootprintSeedTest(unittest.TestCase):
    def test_narrow_straight_seed_reaches_audited_analytic_solution(self):
        seed = straight_path(4)
        request = make_request(seed.start_qf, seed.endpoint())
        config = research_planner_config(guidance_mode=FOOTPRINT_GUIDANCE)
        result = plan_virtual_a(map_request(request, narrow_corridor(), planner=config))
        self.assertTrue(result.executable, (result.status, result.reason, result.detail))
        self.assertTrue(result.collision_audit.passed)
        evidence = result.search["footprint_cover"]
        self.assertEqual(evidence["seed_source"], "straight_a")
        self.assertEqual(evidence["refinement"], "ANALYTIC_STRAIGHT_OPTIMUM")
        self.assertEqual(evidence["region_count"], config.intervals * 3)
        self.assertGreater(evidence["obb_rows"], 0)
        self.assertLessEqual(evidence["seed_nlp_residual"], 1e-8)
        self.assertAlmostEqual(result.path.length_m, 4)

    def test_boxes_share_a_seed_witness_at_the_region_change(self):
        grid = narrow_corridor()
        grid.occ[68:] = False
        grid.occ[68:, :80] = True
        grid.occ[75:, 80:] = True
        # Multiple actual Hermite pieces give the region builder two seams.
        seed = straight_path(4)
        path = plan_virtual_a(map_request(
            make_request(seed.start_qf, seed.endpoint()), grid,
            planner=research_planner_config(
                guidance_mode=FOOTPRINT_GUIDANCE, intervals=4))).path
        self.assertIsNotNone(path)
        request = make_request(path.start_qf, path.endpoint())
        regions = build_footprint_regions(
            path, request, grid, 0.05, 0.5, time.monotonic() + 5)
        self.assertTrue(_cover_passes(
            path, regions, request, 0.05, time.monotonic() + 5))
        for vehicle_id in (1, 2, 3):
            assigned = [r for r in regions if r.vehicle_id == vehicle_id]
            for left, right in zip(assigned, assigned[1:]):
                self.assertEqual(left.sigma_end, right.sigma_start)
            self.assertGreater(len({r.box for r in assigned}), 1)

    def test_vehicle_sweep_rejects_occupied_box_away_from_f_route(self):
        grid = narrow_corridor()
        grid.occ[60, 73] = True  # Upper vehicle crosses; F at y=0 stays clear.
        path = straight_path(4)
        request = make_request(path.start_qf, path.endpoint())
        with self.assertRaises(VirtualAContractError) as caught:
            build_footprint_regions(
                path, request, grid, 0.05, 0.5, time.monotonic() + 5)
        self.assertEqual(caught.exception.reason, "FOOTPRINT_BOX_UNCONFIRMED")
        self.assertEqual(caught.exception.location["vehicle_id"], 2)

    def test_rotated_regions_preserve_diagonal_vehicle_sweeps(self):
        resolution = 0.04
        y, x = np.indices((200, 200))
        occupancy = abs(y - x) * resolution / math.sqrt(2) > 1.34
        grid = Grid(occupancy, resolution, -2.0, -2.0)
        yaw = math.pi / 4
        path = VirtualAPath(FormationPose(0.0, 0.0, yaw), straight_profile(4), MODEL)
        request = make_request(path.start_qf, path.endpoint(),
                               start_headings={vehicle_id: yaw for vehicle_id in (1, 2, 3)})
        deadline = time.monotonic() + 5
        regions = build_footprint_regions(path, request, grid, 0.05, 0.5, deadline)
        self.assertTrue(any(region.yaw for region in regions))
        self.assertTrue(_cover_passes(path, regions, request, 0.05, deadline))

    def test_chord_remainder_encloses_rotating_rigid_vehicle_corners(self):
        path = ramp_path(5.0, 0.2)
        request = make_request(path.start_qf, path.endpoint())
        yaw = 0.37
        c, s = math.cos(yaw), math.sin(yaw)
        for offset in MODEL.offsets:
            corners = _corners(request.vehicle_shapes[offset.vehicle_id], 0.05)
            box = _chord_swept_box(path, offset, corners, 2.4, 2.9, yaw)
            for station in np.linspace(2.4, 2.9, 25):
                for x, y in _world_corners(path, station, offset.vehicle_id, corners):
                    self.assertLessEqual(box[0], c * x + s * y)
                    self.assertGreaterEqual(box[2], c * x + s * y)
                    self.assertLessEqual(box[1], -s * x + c * y)
                    self.assertGreaterEqual(box[3], -s * x + c * y)

    def test_solver_inset_keeps_the_seed_feasible(self):
        path = ramp_path(5.0, 0.2)
        request = make_request(path.start_qf, path.endpoint())
        deadline = time.monotonic() + 5
        regions = build_footprint_regions(path, request, empty_grid(), 0.05, 0.5, deadline)
        limits = _refinement_limits(path, regions, request, 0.05)
        self.assertTrue(any(box != region.box for box, region in zip(limits, regions)))
        inset = tuple(replace(region, box=box) for region, box in zip(regions, limits))
        self.assertTrue(_cover_passes(path, inset, request, 0.05, deadline))

    def test_front_vehicle_limit_rejects_legal_reference_curvature(self):
        path = ramp_path(0.4, 0.4)
        request = make_request(path.start_qf, path.endpoint())
        state = path.state_at_s(path.length_m / 2)
        limit = 1 / request.limits.r_min_ackermann
        self.assertLess(abs(path.profile.jet_at(path.length_m / 2).kappa), limit)
        self.assertGreater(abs(state.vehicles[0].curvature), limit)
        self.assertFalse(audit_geometry(request, path).passed)

    def test_hold_keeps_the_existing_static_audits(self):
        pose = straight_path(1).start_qf
        config = research_planner_config(guidance_mode=FOOTPRINT_GUIDANCE)
        result = plan_virtual_a(map_request(
            make_request(pose, pose), narrow_corridor(), planner=config))
        self.assertEqual(result.status, "NO_MOTION", result.detail)
        self.assertTrue(result.collision_audit.passed)
        self.assertEqual(result.plan.duration_s, 0.0)
