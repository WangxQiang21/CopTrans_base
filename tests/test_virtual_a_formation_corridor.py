"""TASK03 Step 1: versioned scope, complete-OBB templates and actual g rows.

The tests use the frozen straight-line request and hand-built large convex
boxes.  They verify the physical OBB map against ``map_spatial_state`` and the
existing static OBB helpers, then prove the same scalar rows enter both the
numeric residual and CasADi ``g`` (including the Jacobian).  No test claims a
continuous three-vehicle certificate; that is Step 4 work.
"""
from dataclasses import replace
import copy
import hashlib
import json
import math
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

try:
    import casadi
except ImportError:  # reported as NOT_RUN in evidence, never silently faked
    casadi = None

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import (
    FormationCollisionChecker,
    VehicleShape,
)
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    ConvexCorridor,
    CorridorChain,
    FORMATION_CORRIDOR_TEMPLATE_SCHEMA,
    FormationCorridorTemplateSegment,
    GeometryResult,
    InitialGuess,
    FORMATION_CORRIDOR_STATUS_F_ONLY_GATE_BLOCKED,
    FORMATION_CORRIDOR_STATUS_TEMPLATE_INSUFFICIENT,
    FORMATION_CORRIDOR_STATUS_UNCONFIRMED,
    FORMATION_CORRIDOR_STATUS_VERIFIED,
    FORMATION_CORRIDOR_STATUS_VIOLATED,
    FORMATION_CORRIDOR_SCOPE_F_ONLY,
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    FORMATION_NORMAL_BRANCH_EPS,
    FORMATION_NORMAL_BRANCH_PENALTY,
    GuideRoute,
    PlannerConfig,
    VirtualAContractError,
    grid_signature,
    planner_config_from_dict,
    request_fingerprint,
    research_planner_config,
    validate_planner_config,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (
    allocate_corridors,
    assess_formation_guide,
    audit_corridor_chain,
    corridor_effective_margin,
    make_corridor_guide,
    obb_polygon,
    obb_support_halfwidth,
)
from fwis_v3_optimizer.formation_continuous_a.formation_certificate import (
    FORMATION_CERTIFICATE_SCHEMA,
    FORMATION_CERTIFICATE_UNCONFIRMED,
    FORMATION_CERTIFICATE_VERIFIED,
    FORMATION_CERTIFICATE_VIOLATED,
    _heading_interval,
    _inflate as inflate_certificate_interval,
    _seal as seal_certificate,
    certify_formation_path,
    verify_formation_certificate,
)
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (
    FORMATION_SAMPLE_AUDIT_CLEAR,
    FORMATION_SAMPLE_AUDIT_UNCONFIRMED,
    FORMATION_SAMPLE_AUDIT_VIOLATED,
    FormationCoverOption,
    _f_choice,
    _guide_pose_at_sigma,
    _region_contains_point,
    audit_formation_template_samples,
    build_formation_corridor_constraint,
    build_formation_template_candidates,
    diagonal_cover,
    finite_cover_options,
    formation_template_from_payload,
    single_region_template,
    validate_formation_template,
    vehicle_frames,
    whole_cover,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (
    _build_casadi_nlp,
    _build_corridor_constraint,
    _constraint_value_offsets,
    _node_route_curvature,
    _numeric_constraint_vector,
    _search_packet,
    solve_open_path_guided,
)
from tests.test_virtual_a_corridor_optimizer import _box
from tests.test_virtual_a_optimizer import request_from_fixture, planner_config
from virtual_a_g4_support import empty_grid

S_LENGTH = 6.0
INTERVALS = 8
GRID_SHA = "0" * 64
CONTEXT_SHA = "c" * 64
MARGIN_M = 1e-5


def _constant_profile(kappa, slope=0.0, length=S_LENGTH, intervals=INTERVALS):
    h = float(length) / int(intervals)
    etas = [float(slope)] * (int(intervals) + 1)
    kappas = [float(kappa)] * (int(intervals) + 1)
    return C2CurvatureProfile.from_knots([
        ControlKnot(index * h, kappas[index], etas[index])
        for index in range(int(intervals) + 1)])


def _request(psi0=0.0, kappa=0.0):
    base = request_from_fixture("STRAIGHT_LINE")
    goal_psi = float(psi0) + float(kappa) * S_LENGTH
    return request_from_fixture(
        "STRAIGHT_LINE",
        start_qf=FormationPose(0.0, 0.0, float(psi0)),
        start_headings={
            vehicle_id: float(psi0) for vehicle_id in base.start_headings},
        goal_qf=FormationPose(
            S_LENGTH * math.cos(float(psi0)),
            S_LENGTH * math.sin(float(psi0)), goal_psi))


def _scene(psi0=0.0, kappa=0.0, quadrature=9):
    request = _request(psi0, kappa)
    chain = CorridorChain(
        regions=(_box(
            -50.0, -50.0, 50.0, 50.0,
            (0.0, 0.0),
            (S_LENGTH * math.cos(psi0), S_LENGTH * math.sin(psi0))),),
        route_source="fixture", grid_sha256=GRID_SHA)
    guide = make_corridor_guide(
        chain, (0.0, 0.0),
        (S_LENGTH * math.cos(psi0), S_LENGTH * math.sin(psi0)))
    allocation = allocate_corridors(chain, guide, INTERVALS)
    config = planner_config(
        guidance_mode="corridor",
        formation_corridor_scope=FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
        corridor_quadrature_points=quadrature,
        solver="ipopt", max_initials=1)
    template = single_region_template(
        request, chain, allocation, guide,
        planning_context_sha256=CONTEXT_SHA, region_indices=0,
        margin_m=MARGIN_M, margin_source="fixture")
    constraint = build_formation_corridor_constraint(
        template, request, chain, config, allocation=allocation,
        guide=guide, planning_context_sha256=CONTEXT_SHA)
    return {
        "request": request,
        "chain": chain,
        "guide": guide,
        "allocation": allocation,
        "config": config,
        "template": template,
        "constraint": constraint,
        "path": VirtualAPath(
            request.start_qf, _constant_profile(kappa), request.model),
    }


def _z(kappa, length=S_LENGTH):
    return np.asarray(
        [float(length)] + [float(kappa)] * (INTERVALS + 1)
        + [0.0] * (INTERVALS + 1), dtype=float)


def _independent_rows(scene):
    """Rows evaluated from map_spatial_state, never from the optimizer."""
    request = scene["request"]
    constraint = scene["constraint"]
    path = scene["path"]
    psi0 = request.start_qf.psi
    cosine, sine = math.cos(psi0), math.sin(psi0)
    rows = []
    for group in constraint.groups:
        for sample in group:
            state = path.state_at_s(sample.sigma * S_LENGTH)
            vehicle = [
                item for item in state.vehicles
                if item.vehicle_id == sample.vehicle_id][0]
            # The optimizer works in the formation-start local frame.
            center_x = cosine * vehicle.x + sine * vehicle.y
            center_y = -sine * vehicle.x + cosine * vehicle.y
            theta = vehicle.theta - psi0
            for vertex in sample.vertices:
                local_x, local_y = vertex.local_vertex
                corner_x = (center_x + math.cos(theta) * local_x
                            - math.sin(theta) * local_y)
                corner_y = (center_y + math.sin(theta) * local_x
                            + math.cos(theta) * local_y)
                for halfspace in vertex.halfspaces:
                    rows.append(
                        halfspace.row[0] * corner_x
                        + halfspace.row[1] * corner_y
                        - halfspace.limit)
    return np.asarray(rows, dtype=float)


def _independent_f_rows(scene):
    """Template F rows evaluated from path state, never from the optimizer."""
    request = scene["request"]
    constraint = scene["constraint"]
    path = scene["path"]
    psi0 = request.start_qf.psi
    cosine, sine = math.cos(psi0), math.sin(psi0)
    rows = []
    for group in constraint.f_groups:
        for sample in group:
            state = path.state_at_s(sample.sigma * path.length_m)
            qf = state.qf
            # The optimizer works in the formation-start local frame.
            point_x = cosine * qf.x + sine * qf.y
            point_y = -sine * qf.x + cosine * qf.y
            for halfspace in sample.halfspaces:
                rows.append(
                    halfspace.row[0] * point_x
                    + halfspace.row[1] * point_y
                    - halfspace.limit)
    return np.asarray(rows, dtype=float)


def _casadi_nlp(request, config, packet, constraint):
    captured = {}

    def fake_nlpsol(name, plugin, nlp, options=None):
        captured["nlp"] = nlp
        return object()

    with mock.patch.object(casadi, "nlpsol", side_effect=fake_nlpsol):
        built = optimizer_module._build_casadi_nlp(
            request, config, packet, corridor_constraint=constraint)
    return captured["nlp"], built


class ScopeContractTests(unittest.TestCase):
    def test_default_and_corridor_payloads_keep_the_historical_hashes(self):
        default = PlannerConfig()
        payload = default.as_dict()
        self.assertNotIn("formation_corridor_scope", payload)
        restored = planner_config_from_dict(json.loads(json.dumps(payload)))
        self.assertEqual(restored.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_F_ONLY)
        self.assertEqual(restored.as_dict(), payload)
        # The research entry factory only opts into corridor/hard; it does
        # not add the new scope to the payload.
        research = research_planner_config()
        self.assertEqual(research.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_F_ONLY)
        self.assertNotIn("formation_corridor_scope", research.as_dict())
        self.assertEqual(
            research.as_dict(),
            PlannerConfig(
                guidance_mode="corridor",
                corridor_enforcement="hard").as_dict())

        union = research_planner_config(
            guidance_mode="corridor",
            corridor_enforcement="hard",
            formation_corridor_scope="THREE_VEHICLE_UNION_V1")
        self.assertEqual(union.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        self.assertEqual(
            union.as_dict()["formation_corridor_scope"],
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        round_trip = planner_config_from_dict(
            json.loads(json.dumps(union.as_dict())))
        self.assertEqual(round_trip.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        self.assertEqual(round_trip.as_dict(), union.as_dict())

    def test_unknown_or_inconsistent_scope_fails_closed(self):
        unknown = PlannerConfig(formation_corridor_scope="union_v2")
        validation = validate_planner_config(unknown)
        self.assertFalse(validation.passed)
        self.assertEqual(validation.reason, "INVALID_CONFIG")

        legacy = PlannerConfig(formation_corridor_scope=
                               FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        validation = validate_planner_config(legacy)
        self.assertFalse(validation.passed)
        self.assertEqual(validation.reason, "INVALID_CONFIG")

        soft = PlannerConfig(
            guidance_mode="corridor", corridor_enforcement="soft",
            formation_corridor_scope=
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        self.assertEqual(
            validate_planner_config(soft).reason, "INVALID_CONFIG")

        good = PlannerConfig(
            guidance_mode="corridor", corridor_enforcement="hard",
            formation_corridor_scope=
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        self.assertTrue(validate_planner_config(good).passed)

    def test_certificate_status_vocabulary_is_stable(self):
        statuses = (
            FORMATION_CORRIDOR_STATUS_VERIFIED,
            FORMATION_CORRIDOR_STATUS_VIOLATED,
            FORMATION_CORRIDOR_STATUS_UNCONFIRMED,
            FORMATION_CORRIDOR_STATUS_TEMPLATE_INSUFFICIENT,
            FORMATION_CORRIDOR_STATUS_F_ONLY_GATE_BLOCKED,
        )
        self.assertEqual(len(statuses), len(set(statuses)))
        self.assertEqual(FORMATION_CORRIDOR_STATUS_VERIFIED,
                         "FORMATION_CORRIDOR_VERIFIED")
        self.assertEqual(FORMATION_CORRIDOR_STATUS_VIOLATED,
                         "FORMATION_CORRIDOR_VIOLATED")


class TemplateDigestTests(unittest.TestCase):
    def test_digest_is_deterministic_and_covers_every_identity(self):
        scene = _scene()
        template = scene["template"]
        rebuilt = single_region_template(
            scene["request"], scene["chain"], scene["allocation"],
            scene["guide"], planning_context_sha256=CONTEXT_SHA,
            region_indices=0, margin_m=MARGIN_M, margin_source="fixture")
        self.assertEqual(template.template_id, rebuilt.template_id)
        self.assertEqual(template.as_dict(), rebuilt.as_dict())
        payload = template._payload()
        expected = hashlib.sha256(json.dumps(
            payload, sort_keys=True, separators=(",", ":"),
            ensure_ascii=False).encode("utf-8")).hexdigest()
        self.assertEqual(template.template_id, expected)

        for name, value in (
                ("margin_m", MARGIN_M + 1e-9),
                ("route_index", 1),
                ("candidate_index", 2),
                ("planning_context_sha256", "d" * 64),
                ("guide_sha256", "e" * 64),
                ("allocation_digest", "f" * 64),
                ("margin_source", "changed")):
            changed = replace(template, **{name: value})
            self.assertNotEqual(
                changed.template_id, template.template_id,
                "digest did not cover {}".format(name))

    def test_identity_mismatches_are_rejected(self):
        scene = _scene()
        cases = (
            ("request_sha256", "d" * 64),
            ("grid_sha256", "d" * 64),
            ("planning_context_sha256", "d" * 64),
            ("model_sha256", "d" * 64),
            ("chain_digest", "d" * 64),
            ("allocation_digest", "d" * 64),
            ("guide_sha256", "d" * 64),
        )
        for name, value in cases:
            tampered = replace(scene["template"], **{name: value})
            with self.subTest(field=name):
                with self.assertRaises(VirtualAContractError) as raised:
                    validate_formation_template(
                        tampered, scene["request"], scene["chain"],
                        allocation=scene["allocation"],
                        guide=scene["guide"],
                        planning_context_sha256=CONTEXT_SHA)
                self.assertEqual(
                    raised.exception.reason,
                    "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH")

    def test_region_index_and_canonical_vertices_are_checked(self):
        scene = _scene()
        template = scene["template"]
        out_of_chain = replace(
            template,
            segments=(replace(
                template.segments[0],
                covers=tuple(
                    replace(
                        cover,
                        pieces=(replace(cover.pieces[0], region_index=5),))
                    for cover in template.segments[0].covers),),))
        with self.assertRaises(VirtualAContractError) as raised:
            validate_formation_template(
                out_of_chain, scene["request"], scene["chain"],
                allocation=scene["allocation"], guide=scene["guide"],
                planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(
            raised.exception.reason,
            "INVALID_FORMATION_CORRIDOR_TEMPLATE")

        cover = template.segments[0].covers[0]
        bad_piece = replace(
            cover.pieces[0],
            local_vertices=((0.0, 0.0),) + cover.pieces[0].local_vertices[1:])
        with self.assertRaises(VirtualAContractError) as raised:
            replace(
                template,
                segments=(replace(
                    template.segments[0],
                    covers=(replace(cover, pieces=(bad_piece,)),)
                    + template.segments[0].covers[1:]),))
        self.assertIn("INVALID_FORMATION_CORRIDOR_TEMPLATE",
                      (raised.exception.reason,))

    def test_versions_margin_floor_and_region_selectors_fail_closed(self):
        scene = _scene()
        template = scene["template"]
        for change in (
                {"compiler_version": "formation_corridor_rows.v0"},
                {"certificate_version": "formation_corridor_certificate.v0"}):
            with self.subTest(change=change):
                with self.assertRaises(VirtualAContractError) as raised:
                    replace(template, **change)
                self.assertEqual(
                    raised.exception.reason,
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE")

        frames = vehicle_frames(scene["request"])
        for bad in (0.5, True, -1):
            with self.subTest(bad_region=bad):
                with self.assertRaises(VirtualAContractError):
                    whole_cover(frames[0], bad)
        with self.assertRaises(VirtualAContractError):
            diagonal_cover(frames[0], 0.5, 0, 1)

        # A template margin below the active F-corridor margin could make the
        # OBB rows easier than the historical F rows.  It must fail closed.
        tight = replace(
            scene["config"], corridor_margin_m=MARGIN_M * 2.0)
        with self.assertRaises(VirtualAContractError) as raised:
            build_formation_corridor_constraint(
                template, scene["request"], scene["chain"], tight,
                allocation=scene["allocation"], guide=scene["guide"],
                planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(
            raised.exception.reason, "INVALID_FORMATION_CORRIDOR_TEMPLATE")

        # A more conservative OBB buffer remains valid.
        conservative = replace(template, margin_m=MARGIN_M * 10.0)
        built = build_formation_corridor_constraint(
            conservative, scene["request"], scene["chain"], scene["config"],
            allocation=scene["allocation"], guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(built.margin_m, MARGIN_M * 10.0)

    def test_whole_and_diagonal_covers_are_exact_convex_pieces(self):
        scene = _scene()
        frames = vehicle_frames(scene["request"])
        for frame in frames:
            whole = whole_cover(frame, 0)
            self.assertEqual(whole.decomposition, "whole")
            self.assertEqual(len(whole.pieces), 1)
            self.assertEqual(
                tuple(whole.pieces[0].local_vertices),
                tuple(frame.local_corners))
            for diagonal in (0, 1):
                cover = diagonal_cover(frame, diagonal, 0, 1)
                self.assertEqual(cover.decomposition,
                                 "diagonal_{}".format(diagonal))
                self.assertEqual(len(cover.pieces), 2)
                for piece in cover.pieces:
                    xs = [vertex[0] for vertex in piece.local_vertices]
                    ys = [vertex[1] for vertex in piece.local_vertices]
                    area = 0.5 * (
                        sum(xs[i] * ys[(i + 1) % 3] - xs[(i + 1) % 3] * ys[i]
                            for i in range(3)))
                    self.assertGreater(area, 0.0)


class ObbGeometryTests(unittest.TestCase):
    def test_four_headings_and_signed_curvature_match_map_state(self):
        combinations = (
            (0.0, 0.1), (0.7, -0.1), (-1.1, 0.1), (2.6, -0.1))
        for psi0, kappa in combinations:
            with self.subTest(psi0=psi0, kappa=kappa):
                scene = _scene(psi0=psi0, kappa=kappa)
                request = scene["request"]
                constraint = scene["constraint"]
                path = scene["path"]
                cosine, sine = math.cos(psi0), math.sin(psi0)
                center_error = 0.0
                heading_error = 0.0
                support_error = 0.0
                polygon_error = 0.0
                for group in constraint.groups:
                    for sample in group:
                        state = path.state_at_s(sample.sigma * S_LENGTH)
                        vehicle = [
                            item for item in state.vehicles
                            if item.vehicle_id == sample.vehicle_id][0]
                        qf = state.qf
                        kappa_at = state.curvature.kappa
                        forward = 1.0 - sample.offset_y * kappa_at
                        lateral = sample.offset_x * kappa_at
                        theta = qf.psi + math.atan2(lateral, forward)
                        center_x = (
                            qf.x + math.cos(qf.psi) * sample.offset_x
                            - math.sin(qf.psi) * sample.offset_y)
                        center_y = (
                            qf.y + math.sin(qf.psi) * sample.offset_x
                            + math.cos(qf.psi) * sample.offset_y)
                        local_x = cosine * center_x + sine * center_y
                        local_y = -sine * center_x + cosine * center_y
                        local_theta = theta - psi0
                        center_error = max(center_error, math.hypot(
                            local_x - (cosine * vehicle.x + sine * vehicle.y),
                            local_y - (-sine * vehicle.x
                                       + cosine * vehicle.y)))
                        heading_error = max(
                            heading_error,
                            abs(local_theta - (vehicle.theta - psi0)))

                        shape = request.vehicle_shapes[sample.vehicle_id]
                        corners = [
                            vertex.local_vertex for vertex in sample.vertices]
                        actual_polygon = tuple(
                            (center_x + math.cos(theta) * px
                             - math.sin(theta) * py,
                             center_y + math.sin(theta) * px
                             + math.cos(theta) * py)
                            for px, py in corners)
                        reference_polygon = obb_polygon(
                            (center_x, center_y), theta, shape)
                        for actual, reference in zip(
                                actual_polygon, reference_polygon):
                            polygon_error = max(
                                polygon_error,
                                math.hypot(actual[0] - reference[0],
                                           actual[1] - reference[1]))
                        for halfspace_id in range(
                                len(sample.vertices[0].halfspaces)):
                            halfspace = sample.vertices[0].halfspaces[
                                halfspace_id]
                            projections = []
                            for vertex in sample.vertices:
                                px, py = vertex.local_vertex
                                corner_x = local_x \
                                    + math.cos(local_theta) * px \
                                    - math.sin(local_theta) * py
                                corner_y = local_y \
                                    + math.sin(local_theta) * px \
                                    + math.cos(local_theta) * py
                                projections.append(
                                    halfspace.row[0] * corner_x
                                    + halfspace.row[1] * corner_y)
                            center_projection = (
                                halfspace.row[0] * local_x
                                + halfspace.row[1] * local_y)
                            expected = center_projection \
                                + obb_support_halfwidth(
                                    halfspace.row, local_theta, shape)
                            support_error = max(
                                support_error,
                                abs(max(projections) - expected))
                self.assertLessEqual(center_error, 1e-12)
                self.assertLessEqual(heading_error, 1e-12)
                self.assertLessEqual(polygon_error, 1e-12)
                self.assertLessEqual(support_error, 1e-10)

    def test_compiled_rows_equal_independent_map_geometry(self):
        scene = _scene(psi0=0.7, kappa=-0.1)
        request = scene["request"]
        config = scene["config"]
        constraint = scene["constraint"]
        packet = _search_packet(request, config)
        vector = _z(-0.1)
        wrapper = _CorridorWrapper(constraint)
        equality_end, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, wrapper)
        self.assertGreater(equality_end, 0)
        self.assertLess(equality_end, signed_end)
        f_count = constraint.formation_reference_constraint_count
        self.assertGreater(f_count, 0)
        self.assertEqual(corridor_end - signed_end,
                         f_count + constraint.scalar_row_count)
        values = _numeric_constraint_vector(
            vector, request, config, packet,
            int(config.collocation_points),
            corridor_constraint=_CorridorWrapper(constraint))
        f_values = values[
            corridor_end - constraint.scalar_row_count - f_count:
            corridor_end - constraint.scalar_row_count]
        formation_values = values[
            corridor_end - constraint.scalar_row_count:corridor_end]
        independent_f = _independent_f_rows(scene)
        independent = _independent_rows(scene)
        self.assertEqual(f_values.size, independent_f.size)
        self.assertEqual(formation_values.size, independent.size)
        np.testing.assert_allclose(
            f_values, independent_f, atol=1e-8, rtol=0.0)
        np.testing.assert_allclose(
            formation_values, independent, atol=1e-8, rtol=0.0)

    def test_each_corner_carries_every_assigned_region_halfspace(self):
        scene = _scene(psi0=0.7, kappa=-0.1)
        request = scene["request"]
        chain = scene["chain"]
        constraint = scene["constraint"]
        margin = float(scene["template"].margin_m)
        self.assertGreaterEqual(
            margin + 1e-12,
            float(corridor_effective_margin(chain, scene["config"])))
        start = request.start_qf
        cosine, sine = math.cos(start.psi), math.sin(start.psi)
        for group in constraint.groups:
            for sample in group:
                region = chain.regions[sample.body_region_index]
                self.assertEqual(len(sample.vertices), 4)
                self.assertEqual(
                    [vertex.vertex_id for vertex in sample.vertices],
                    [0, 1, 2, 3])
                for vertex in sample.vertices:
                    self.assertEqual(
                        [halfspace.halfspace_id
                         for halfspace in vertex.halfspaces],
                        list(range(len(region.A))))
                    for halfspace in vertex.halfspaces:
                        self.assertEqual(
                            halfspace.region_index,
                            sample.body_region_index)
                        source_id = halfspace.halfspace_id
                        self.assertEqual(
                            halfspace.row,
                            (float(region.A[source_id][0] * cosine
                                   + region.A[source_id][1] * sine),
                             float(-region.A[source_id][0] * sine
                                   + region.A[source_id][1] * cosine)))
                        self.assertEqual(
                            halfspace.limit,
                            float(region.b[source_id]) - margin
                            - (float(region.A[source_id][0]) * start.x
                               + float(region.A[source_id][1]) * start.y))

    def test_actual_row_counts_and_provenance_cover_every_vehicle(self):
        scene = _scene()
        constraint = scene["constraint"]
        self.assertEqual(
            constraint.sample_count,
            (INTERVALS * 9 + INTERVALS + 1) * 3)
        self.assertEqual(constraint.scalar_row_count,
                         constraint.sample_count * 4 * 4)
        self.assertEqual(dict(constraint.rows_by_vehicle), {
            1: constraint.scalar_row_count // 3,
            2: constraint.scalar_row_count // 3,
            3: constraint.scalar_row_count // 3})
        references = constraint.row_references()
        self.assertEqual(len(references), constraint.scalar_row_count)
        self.assertEqual(
            [row["g_index"] for row in references],
            list(range(constraint.scalar_row_count)))
        self.assertEqual({row["vehicle_id"] for row in references},
                         {1, 2, 3})
        self.assertEqual({row["decomposition"] for row in references},
                         {"whole"})
        self.assertEqual({row["region_id"] for row in references}, {0})


class _CorridorWrapper:
    """Adapt a bare formation plan to the optimizer's normal carrier."""

    def __init__(self, formation):
        self.groups = tuple(() for _ in range(INTERVALS))
        self.formation = formation


class NormalBranchTests(unittest.TestCase):
    def test_degenerate_vehicle_maps_are_infeasible_in_numeric_and_casadi(self):
        if casadi is None:
            self.skipTest("CasADi not installed")
        scene = _scene()
        request = scene["request"]
        config = scene["config"]
        constraint = _CorridorWrapper(scene["constraint"])
        packet = _search_packet(request, config)
        _equality_end, _signed_end, corridor_end = \
            _constraint_value_offsets(request, config, packet, constraint)
        count = scene["constraint"].scalar_row_count
        nlp, _built = _casadi_nlp(request, config, packet, constraint)
        g_function = casadi.Function("g", [nlp["x"]], [nlp["g"]])

        # Vehicle 2 has b=+1 (A=1-kappa); vehicle 3 has b=-1
        # (A=1+kappa).  kappa=1/-1 makes A=D=0; kappa=2/-2 makes A<0.
        for kappa, vehicle_id in (
                (1.0, 2), (2.0, 2), (-1.0, 3), (-2.0, 3)):
            with self.subTest(kappa=kappa, vehicle_id=vehicle_id):
                vector = _z(kappa)
                numeric = _numeric_constraint_vector(
                    vector, request, config, packet,
                    int(config.collocation_points),
                    corridor_constraint=constraint)
                symbolic = np.asarray(
                    g_function(vector), dtype=float).ravel()
                numeric_rows = numeric[
                    corridor_end - count:corridor_end]
                symbolic_rows = symbolic[
                    corridor_end - count:corridor_end]
                self.assertTrue(np.all(np.isfinite(numeric_rows)))
                self.assertTrue(np.all(np.isfinite(symbolic_rows)))
                np.testing.assert_allclose(
                    numeric_rows, symbolic_rows, atol=1e-3, rtol=0.0)
                references = scene["constraint"].row_references()
                degenerate = []
                for index, reference in enumerate(references):
                    if reference["vehicle_id"] == vehicle_id:
                        degenerate.append(numeric_rows[index])
                        self.assertGreater(
                            numeric_rows[index],
                            0.5 * FORMATION_NORMAL_BRANCH_PENALTY,
                            (kappa, index, reference))
                self.assertTrue(degenerate)


class OptimizerRowTests(unittest.TestCase):
    def test_numeric_and_casadi_g_share_every_scalar_row(self):
        if casadi is None:
            self.skipTest("CasADi not installed")
        scene = _scene(psi0=0.0, kappa=0.1)
        request = scene["request"]
        config = scene["config"]
        packet = _search_packet(request, config)
        constraint = scene["constraint"]
        wrapper = _CorridorWrapper(constraint)
        vector = _z(0.1)
        equality_end, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, wrapper)
        f_count = constraint.formation_reference_constraint_count
        self.assertGreater(f_count, 0)
        self.assertEqual(corridor_end - signed_end,
                         f_count + constraint.scalar_row_count)
        numeric = _numeric_constraint_vector(
            vector, request, config, packet,
            int(config.collocation_points), corridor_constraint=wrapper)
        nlp, built = _casadi_nlp(request, config, packet, wrapper)
        _solver, _lower_x, _upper_x, lower_g, upper_g = built
        g_function = casadi.Function("g", [nlp["x"]], [nlp["g"]])
        symbolic = np.asarray(g_function(vector), dtype=float).ravel()
        self.assertEqual(symbolic.size, corridor_end)
        self.assertEqual(numeric.size, corridor_end)
        np.testing.assert_allclose(
            symbolic, numeric, atol=1e-8, rtol=0.0)
        formation_slice = slice(corridor_end - constraint.scalar_row_count,
                                corridor_end)
        f_slice = slice(corridor_end - constraint.scalar_row_count - f_count,
                        corridor_end - constraint.scalar_row_count)
        self.assertTrue(np.all(np.isneginf(lower_g[formation_slice])))
        self.assertTrue(np.all(upper_g[formation_slice] == 0.0))
        self.assertTrue(np.all(np.isneginf(lower_g[f_slice])))
        self.assertTrue(np.all(upper_g[f_slice] == 0.0))
        self.assertEqual(
            int(np.count_nonzero(np.isfinite(symbolic[formation_slice]))),
            constraint.scalar_row_count)
        self.assertEqual(
            int(np.count_nonzero(np.isfinite(symbolic[f_slice]))),
            f_count)

    def test_casadi_jacobian_matches_finite_differences(self):
        if casadi is None:
            self.skipTest("CasADi not installed")
        scene = _scene()
        request = scene["request"]
        config = scene["config"]
        packet = _search_packet(request, config)
        constraint = scene["constraint"]
        wrapper = _CorridorWrapper(constraint)
        vector = _z(0.0)
        _equality_end, _signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, wrapper)
        nlp, _built = _casadi_nlp(request, config, packet, wrapper)
        jacobian = casadi.Function(
            "jacobian", [nlp["x"]], [casadi.jacobian(nlp["g"], nlp["x"])])
        analytic = np.asarray(jacobian(vector), dtype=float)
        formation_slice = slice(
            corridor_end - constraint.scalar_row_count, corridor_end)
        step = 1e-6
        for column in (0, 1, 2, 9, 10, 18):
            plus = vector.copy()
            minus = vector.copy()
            plus[column] += step
            minus[column] -= step
            plus_values = _numeric_constraint_vector(
                plus, request, config, packet,
                int(config.collocation_points),
                corridor_constraint=wrapper)[formation_slice]
            minus_values = _numeric_constraint_vector(
                minus, request, config, packet,
                int(config.collocation_points),
                corridor_constraint=wrapper)[formation_slice]
            finite_difference = (plus_values - minus_values) / (2.0 * step)
            expected = analytic[formation_slice, column]
            with self.subTest(column=column):
                np.testing.assert_allclose(
                    finite_difference, expected, atol=1e-7, rtol=1e-5)
                finite = finite_difference[np.abs(expected) > 1e-3]
                if finite.size:
                    relative = np.abs(
                        (finite - expected[np.abs(expected) > 1e-3])
                        / finite)
                    self.assertLessEqual(float(np.max(relative)), 1e-5)

    def test_solve_gates_union_scope_on_the_selected_template(self):
        scene = _scene()
        request = scene["request"]
        config = scene["config"]
        base_kwargs = {
            "route_guides": (scene["guide"],),
            "corridor_chain": scene["chain"],
            "corridor_allocation": scene["allocation"],
        }
        no_template = solve_open_path_guided(request, config, **base_kwargs)
        self.assertEqual(no_template.status, "INVALID_REQUEST")
        self.assertEqual(no_template.reason,
                         "FORMATION_CORRIDOR_TEMPLATE_REQUIRED")

        f_only = solve_open_path_guided(
            request,
            replace_config_scope(scene["config"], "f_only"),
            formation_template=scene["template"],
            formation_planning_context_sha256=CONTEXT_SHA,
            **base_kwargs)
        self.assertEqual(f_only.status, "INVALID_REQUEST")
        self.assertEqual(f_only.reason,
                         "FORMATION_CORRIDOR_SCOPE_MISMATCH")

        wrong_context = solve_open_path_guided(
            request, config,
            formation_template=scene["template"],
            formation_planning_context_sha256="d" * 64,
            **base_kwargs)
        self.assertEqual(wrong_context.status, "INVALID_REQUEST")
        self.assertEqual(
            wrong_context.reason,
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH")

    def test_union_scope_solve_reports_actual_template_rows(self):
        scene = _scene()
        request = scene["request"]
        config = replace_config_scope(
            scene["config"], FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
            max_seconds=20.0)
        result = solve_open_path_guided(
            request, config,
            route_guides=(scene["guide"],),
            corridor_chain=scene["chain"],
            corridor_allocation=scene["allocation"],
            formation_template=scene["template"],
            formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertTrue(result.audit.passed)
        self.assertEqual(result.formation_corridor_scope,
                         FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)
        self.assertEqual(result.formation_template_id,
                         scene["template"].template_id)
        self.assertEqual(result.formation_obb_constraint_count,
                         scene["constraint"].scalar_row_count)
        self.assertEqual(
            result.formation_f_constraint_count,
            scene["constraint"].formation_reference_constraint_count)
        # Under union scope the historical fixed-allocation F-only rows are
        # not compiled at all; F is represented by the template rows.
        self.assertEqual(result.corridor_halfspace_constraint_count, 0)
        self.assertEqual(
            result.as_dict()["formation_obb_constraint_count"],
            scene["constraint"].scalar_row_count)
        self.assertEqual(
            result.as_dict()["formation_f_constraint_count"],
            scene["constraint"].formation_reference_constraint_count)
        self.assertIsNotNone(result.corridor_max_residual_m)
        self.assertLessEqual(result.corridor_max_residual_m, 1e-8)
        self.assertIsNotNone(result.formation_f_max_residual_m)
        self.assertLessEqual(result.formation_f_max_residual_m, 1e-8)


# ---------------------------------------------------------------------------
# Step 2: finite whole/diagonal Portal template candidates
# ---------------------------------------------------------------------------

PORTAL_THETA = math.pi / 2.0 - math.atan2(0.500, 0.738)


def _world_vertices(local_vertices, center, theta):
    cosine, sine = math.cos(theta), math.sin(theta)
    return tuple(
        (center[0] + cosine * x - sine * y,
         center[1] + sine * x + cosine * y)
        for x, y in local_vertices)


def _point_in_region(point, region):
    return all(
        row[0] * point[0] + row[1] * point[1] <= limit + 1e-12
        for row, limit in zip(region.A, region.b))


def _portal_corridor(x0, y0, x1, y1, seed_start, seed_end):
    return _box(x0, y0, x1, y1, seed_start, seed_end)


def _assess_portal(chain, guide, request, grid, allocation,
                   start_heading, goal_heading, **changes):
    checker = FormationCollisionChecker(
        grid, request.model, request.vehicle_shapes, 0.05)
    curvature = tuple(
        float(value) for value in _node_route_curvature(guide.points))
    options = {
        "route_index": 0,
        "round_index": 0,
        "guide_source": guide.source,
        "allocation": allocation,
        "node_curvature": curvature,
        "node_curvature_valid": tuple(True for _ in curvature),
        "grid_sha256": grid_signature(grid),
        "request_sha256": request_fingerprint(request),
        "planning_context_sha256": CONTEXT_SHA,
    }
    options.update(changes)
    return assess_formation_guide(
        chain, guide.points, checker, request.model, request.vehicle_shapes,
        start_heading, goal_heading, **options)


def _portal_scene(overlap_half=0.3):
    """Two real audited overlapping boxes and a body-diagonal guide.

    ``overlap_half`` is the half-width of the overlap.  At 0.3 m the finite
    whole/diagonal covers can express the complete transition; at 0.2 m a
    real union-covered OBB pose falls outside every finite template, which is
    the Step 2 COVER_TEMPLATE_INSUFFICIENT witness.  The guide heading is the
    direction that puts one rectangle diagonal exactly on the vertical
    overlap boundary.
    """
    grid = empty_grid(
        width=400, height=200, origin_x=-5.0, origin_y=-5.0)
    grid_sha = grid_signature(grid)
    distance = 2.0
    start = (-distance * math.cos(PORTAL_THETA),
             -distance * math.sin(PORTAL_THETA))
    middle = (0.0, 0.0)
    goal = (distance * math.cos(PORTAL_THETA),
            distance * math.sin(PORTAL_THETA))
    chain = CorridorChain(
        regions=(
            _portal_corridor(
                -2.5, -3.0, overlap_half, 3.0, start, middle),
            _portal_corridor(
                -overlap_half, -3.0, 2.5, 3.0, middle, goal)),
        route_source="task03_step2_portal_fixture",
        grid_sha256=grid_sha)
    audit = audit_corridor_chain(grid, chain)
    if not audit["passed"]:
        raise AssertionError(
            "Step 2 fixture chain must audit: {} {}".format(
                audit["reason"], audit["detail"]))
    guide = GuideRoute(
        points=(start, middle, goal), source="task03_step2_guide",
        grid_sha256=grid_sha)
    allocation = allocate_corridors(chain, guide, 8)
    request = request_from_fixture("STRAIGHT_LINE")
    assessment = _assess_portal(
        chain, guide, request, grid, allocation,
        PORTAL_THETA, PORTAL_THETA)
    config = planner_config(
        guidance_mode="corridor",
        formation_corridor_scope=(
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1),
        corridor_quadrature_points=9, solver="ipopt", max_initials=1)
    return {
        "grid": grid,
        "chain": chain,
        "chain_audit": audit,
        "guide": guide,
        "allocation": allocation,
        "request": request,
        "assessment": assessment,
        "config": config,
    }


def _build_portal_candidates(scene, request=None, chain=None, guide=None,
                             assessment=None):
    return build_formation_template_candidates(
        scene["request"] if request is None else request,
        scene["chain"] if chain is None else chain,
        scene["guide"] if guide is None else guide,
        scene["allocation"],
        scene["assessment"] if assessment is None else assessment,
        scene["config"],
        planning_context_sha256=CONTEXT_SHA,
        margin_m=MARGIN_M, margin_source="task03_step2_fixture")


class PortalFiniteCoverTests(unittest.TestCase):
    """Finite cover predicates at one pose: whole, diagonal and gap."""

    def test_three_vehicles_whole_diagonal_and_nonconvex_gap(self):
        request = _request()
        frame = vehicle_frames(request)[0]
        alpha = math.atan2(frame.width_m, frame.length_m)

        # E07's fixed fixture: all four OBB corners are in one of the two
        # separated convex strips, but the rectangle interior (its centre)
        # is in the nonconvex union's gap.  Corner membership never suffices.
        upper = _box(-0.30, 0.32, 0.30, 0.45, (0.0, 0.32), (0.0, 0.45))
        lower = _box(-0.30, -0.45, 0.30, -0.32, (0.0, -0.45), (0.0, -0.32))
        gap_options = finite_cover_options(
            frame, (0.0, 0.0), math.pi / 2.0, (upper, lower), (0, 1))
        self.assertEqual(gap_options, ())
        corners = _world_vertices(
            frame.local_corners, (0.0, 0.0), math.pi / 2.0)
        self.assertTrue(all(
            _point_in_region(corner, upper) or
            _point_in_region(corner, lower) for corner in corners))
        self.assertFalse(_point_in_region((0.0, 0.0), upper))
        self.assertFalse(_point_in_region((0.0, 0.0), lower))

        # Adjacent regions may hold different whole vehicles.
        left = _box(-10.0, -10.0, 0.0, 10.0, (-1.0, 0.0), (0.0, 0.0))
        right = _box(0.0, -10.0, 10.0, 10.0, (0.0, 0.0), (1.0, 0.0))
        self.assertIn(
            FormationCoverOption("whole", (0,)),
            finite_cover_options(
                frame, (-2.0, 0.0), 0.0, (left, right), (0, 1)))
        self.assertIn(
            FormationCoverOption("whole", (1,)),
            finite_cover_options(
                frame, (2.0, 0.0), 0.0, (left, right), (0, 1)))

        # With the rectangle diagonal on the artificial boundary, each
        # canonical triangle lies wholly in a different adjacent region.
        diag_zero = finite_cover_options(
            frame, (0.0, 0.0), math.pi / 2.0 - alpha,
            (left, right), (0, 1))
        self.assertEqual(
            diag_zero,
            (FormationCoverOption("diagonal_0", (1, 0)),))
        diag_one = finite_cover_options(
            frame, (0.0, 0.0), math.pi / 2.0 + alpha,
            (left, right), (0, 1))
        self.assertEqual(
            diag_one,
            (FormationCoverOption("diagonal_1", (0, 1)),))
        for cover in (diagonal_cover(frame, 0, 1, 0),
                      diagonal_cover(frame, 1, 0, 1)):
            for piece in cover.pieces:
                for vertex in _world_vertices(
                        piece.local_vertices, (0.0, 0.0),
                        math.pi / 2.0 - alpha
                        if cover.decomposition == "diagonal_0"
                        else math.pi / 2.0 + alpha):
                    region = right if piece.region_index == 1 else left
                    self.assertTrue(_point_in_region(vertex, region))

        # A rectangle crossing a vertical boundary along its long axis has
        # no expressible finite cover even though it is a real rectangle.
        self.assertEqual(
            finite_cover_options(
                frame, (0.0, 0.0), 0.0, (left, right), (0, 1)), ())


class PortalTemplateBuilderTests(unittest.TestCase):
    def test_real_audited_chain_builds_whole_and_diagonal_segments(self):
        scene = _portal_scene(overlap_half=0.3)
        self.assertTrue(scene["chain_audit"]["passed"])
        assessment = scene["assessment"]
        self.assertEqual(assessment["map_sample_status"],
                         "CLEAR_AT_SAMPLES")
        self.assertEqual(assessment["map_sample_counts"]["map_collision"],
                         0)
        self.assertEqual(assessment["candidate_certification"],
                         "DIAGNOSTIC_ONLY")
        self.assertFalse(assessment["continuously_certified"])
        self.assertFalse(assessment["formation_obb_hard_constraints"])

        candidates = _build_portal_candidates(scene)
        self.assertGreaterEqual(len(candidates.templates), 1)
        self.assertLessEqual(len(candidates.templates), 3)
        self.assertEqual(candidates.cuts[0], 0.0)
        self.assertEqual(candidates.cuts[-1], 1.0)
        self.assertGreater(candidates.station_count, 0)
        template = candidates.templates[0]
        validate_formation_template(
            template, scene["request"], scene["chain"],
            allocation=scene["allocation"], guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)

        vehicle_ids = {frame.vehicle_id
                       for frame in vehicle_frames(scene["request"])}
        diagonal_count = 0
        distinct_assignment_segments = 0
        for segment in template.segments:
            self.assertEqual(
                {cover.vehicle_id for cover in segment.covers}, vehicle_ids)
            assignments = set()
            for cover in segment.covers:
                if cover.decomposition != "whole":
                    diagonal_count += 1
                assignments.add(tuple(
                    piece.region_index for piece in cover.pieces))
            if len(assignments) > 1:
                distinct_assignment_segments += 1
        self.assertGreater(diagonal_count, 0)
        self.assertGreater(distinct_assignment_segments, 0)
        for previous, current in zip(
                template.segments, template.segments[1:]):
            self.assertAlmostEqual(
                previous.sigma_end, current.sigma_start, places=12)
        self.assertAlmostEqual(template.segments[0].sigma_start, 0.0,
                               places=12)
        self.assertAlmostEqual(template.segments[-1].sigma_end, 1.0,
                               places=12)

        built = build_formation_corridor_constraint(
            template, scene["request"], scene["chain"], scene["config"],
            allocation=scene["allocation"], guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(built.template_id, template.template_id)
        self.assertGreater(built.scalar_row_count, 0)
        self.assertEqual(
            built.scalar_row_count,
            sum(sample.scalar_row_count
                for group in built.groups for sample in group))
        self.assertEqual(
            built.sample_count,
            sum(len(group) for group in built.groups))
        self.assertEqual(
            {vehicle_id for vehicle_id, count in built.rows_by_vehicle},
            vehicle_ids)
        self.assertTrue(all(count > 0
                            for _vehicle_id, count in built.rows_by_vehicle))
        self.assertTrue(all(count > 0
                            for count in built.segment_row_counts))

        # The first internal cut is shared by the two neighbouring fixed
        # assignments; both must be present and each sample must use its own
        # segment's cover, not one side silently dropped.
        boundary = template.segments[0].sigma_end
        boundary_samples = [
            sample for group in built.groups for sample in group
            if abs(sample.sigma - boundary) <= 1e-12]
        self.assertTrue(boundary_samples)
        self.assertEqual(
            {sample.template_segment_index for sample in boundary_samples},
            {0, 1})
        self.assertEqual(
            {sample.source for sample in boundary_samples},
            {"template_segment_boundary"})
        self.assertEqual(
            {sample.body_region_index for sample in boundary_samples},
            {0, 1})
        for sample in boundary_samples:
            cover = next(
                cover for cover in template.segments[
                    sample.template_segment_index].covers
                if cover.vehicle_id == sample.vehicle_id)
            self.assertEqual(sample.decomposition, cover.decomposition)
            self.assertEqual(
                sample.body_region_index,
                cover.pieces[sample.body_piece_index].region_index)

        # Same selected candidate state and same inputs are deterministic.
        again = _build_portal_candidates(scene)
        self.assertEqual([item.template_id for item in again.templates],
                         [item.template_id for item in candidates.templates])
        self.assertEqual(again.as_dict(), candidates.as_dict())

        # The template digest binds every copied identity field.
        for name, value in (
                ("guide_sha256", "d" * 64),
                ("allocation_digest", "d" * 64),
                ("chain_digest", "d" * 64),
                ("grid_sha256", "d" * 64),
                ("model_sha256", "d" * 64),
                ("request_sha256", "d" * 64),
                ("margin_m", MARGIN_M + 1e-9),
                ("margin_source", "changed")):
            with self.subTest(field=name):
                changed = replace(template, **{name: value})
                self.assertNotEqual(
                    changed.template_id, template.template_id)

    def test_digest_changes_with_actual_guide_chain_grid_and_model(self):
        scene = _portal_scene(overlap_half=0.3)
        base_id = _build_portal_candidates(scene).templates[0].template_id

        guide = replace(
            scene["guide"], source="task03_step2_guide_changed")
        assessment = _assess_portal(
            scene["chain"], guide, scene["request"], scene["grid"],
            scene["allocation"], PORTAL_THETA, PORTAL_THETA)
        changed_guide = _build_portal_candidates(
            scene, guide=guide, assessment=assessment)
        self.assertNotEqual(
            changed_guide.templates[0].template_id, base_id)

        chain = replace(
            scene["chain"],
            route_source="task03_step2_chain_changed")
        assessment = _assess_portal(
            chain, scene["guide"], scene["request"], scene["grid"],
            scene["allocation"], PORTAL_THETA, PORTAL_THETA)
        changed_chain = _build_portal_candidates(
            scene, chain=chain, assessment=assessment)
        self.assertNotEqual(
            changed_chain.templates[0].template_id, base_id)

        other_grid = empty_grid(
            width=400, height=200, origin_x=-5.0, origin_y=-5.0)
        other_grid.occ[0, 0] = True
        other_sha = grid_signature(other_grid)
        grid_chain = replace(scene["chain"], grid_sha256=other_sha)
        grid_guide = replace(scene["guide"], grid_sha256=other_sha)
        assessment = _assess_portal(
            grid_chain, grid_guide, scene["request"], other_grid,
            scene["allocation"], PORTAL_THETA, PORTAL_THETA)
        changed_grid = _build_portal_candidates(
            scene, chain=grid_chain, guide=grid_guide,
            assessment=assessment)
        self.assertNotEqual(
            changed_grid.templates[0].template_id, base_id)
        self.assertEqual(changed_grid.templates[0].grid_sha256, other_sha)

        shapes = {
            vehicle_id: VehicleShape(0.740, 0.500)
            for vehicle_id in scene["request"].vehicle_shapes}
        shape_request = request_from_fixture(
            "STRAIGHT_LINE", vehicle_shapes=shapes)
        assessment = _assess_portal(
            scene["chain"], scene["guide"], shape_request, scene["grid"],
            scene["allocation"], PORTAL_THETA, PORTAL_THETA)
        changed_shape = _build_portal_candidates(
            scene, request=shape_request, assessment=assessment)
        self.assertNotEqual(
            changed_shape.templates[0].template_id, base_id)
        self.assertNotEqual(
            changed_shape.templates[0].model_sha256,
            _build_portal_candidates(scene).templates[0].model_sha256)

    def test_insufficient_template_is_unknown_not_physical_infeasibility(self):
        scene = _portal_scene(overlap_half=0.2)
        assessment = scene["assessment"]
        self.assertTrue(scene["chain_audit"]["passed"])
        self.assertEqual(assessment["map_sample_status"],
                         "CLEAR_AT_SAMPLES")
        self.assertEqual(assessment["map_sample_counts"]["map_collision"],
                         0)
        with self.assertRaises(VirtualAContractError) as raised:
            _build_portal_candidates(scene)
        self.assertEqual(
            raised.exception.reason, "COVER_TEMPLATE_INSUFFICIENT")
        self.assertNotIn("MAP_OBB_COLLISION", str(raised.exception))
        self.assertNotIn("physical", str(raised.exception).lower())


class PortalAssessmentGateTests(unittest.TestCase):
    def test_fake_assessment_identity_map_and_claims_fail_closed(self):
        scene = _portal_scene(overlap_half=0.3)
        assessment = scene["assessment"]
        cases = (
            ("candidate_id", "0" * 64,
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("request_sha256", "0" * 64,
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("grid_sha256", "0" * 64,
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("chain_digest", "0" * 64,
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("planning_context_sha256", "0" * 64,
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("selected_guide_source", "other",
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("selected_guide_sha256", "0" * 64,
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("legacy_f_allocation", [[0.0, 1.0, 0]],
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("schema", "virtual_a_formation_assessment_v0",
             "INVALID_FORMATION_ASSESSMENT"),
            ("candidate_certification", "CERTIFIED",
             "INVALID_FORMATION_ASSESSMENT"),
            ("formation_obb_hard_constraints", True,
             "INVALID_FORMATION_ASSESSMENT"),
            ("continuously_certified", True,
             "INVALID_FORMATION_ASSESSMENT"),
            ("route_index", None,
             "INVALID_FORMATION_ASSESSMENT"),
            ("sample_cap", True,
             "FORMATION_CORRIDOR_UNCONFIRMED"),
            ("dropped_required_stations", 1,
             "FORMATION_CORRIDOR_UNCONFIRMED"),
            ("incomplete_reasons", ["MISSING_VEHICLE_SHAPE"],
             "FORMATION_CORRIDOR_UNCONFIRMED"),
            ("station_coverage", [],
             "FORMATION_CORRIDOR_UNCONFIRMED"),
            ("route_length_m", -1.0,
             "INVALID_FORMATION_ASSESSMENT"),
            ("route_length_m", 1e9,
             "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH"),
            ("station_count", 0,
             "INVALID_FORMATION_ASSESSMENT"),
            ("sampled_station_count", 0,
             "INVALID_FORMATION_ASSESSMENT"),
        )
        for field, value, reason in cases:
            with self.subTest(field=field):
                tampered = copy.deepcopy(assessment)
                tampered[field] = value
                with self.assertRaises(VirtualAContractError) as raised:
                    _build_portal_candidates(scene, assessment=tampered)
                self.assertEqual(raised.exception.reason, reason)
        def _duplicate_station(item):
            item["station_coverage"].append(
                copy.deepcopy(item["station_coverage"][0]))
            item["station_count"] = len(item["station_coverage"])
            item["sampled_station_count"] = len(item["station_coverage"])

        for mutate, label, reason in (
                (lambda item: item["station_coverage"][0].__setitem__(
                    "map_status", "COLLISION"), "map_status",
                 "FORMATION_CORRIDOR_UNCONFIRMED"),
                (lambda item: item["station_coverage"][0].__setitem__(
                    "map_status", "UNCONFIRMED"), "map_unconfirmed",
                 "FORMATION_CORRIDOR_UNCONFIRMED"),
                (lambda item: item["station_coverage"][0]["guide_pose"]
                 .__setitem__("pose_reliable", False), "pose_unreliable",
                 "FORMATION_CORRIDOR_UNCONFIRMED"),
                (lambda item: item["station_coverage"][0]["guide_pose"]
                 .__setitem__("heading_rad", None), "heading_missing",
                 "FORMATION_CORRIDOR_UNCONFIRMED"),
                (lambda item: item["station_coverage"][0].__setitem__(
                    "sigma", 1.5), "sigma_out_of_range",
                 "INVALID_FORMATION_ASSESSMENT"),
                (lambda item: item["station_coverage"][0].__setitem__(
                    "s_m", item["station_coverage"][0]["s_m"] + 0.1),
                 "s_m_inconsistent", "INVALID_FORMATION_ASSESSMENT"),
                (_duplicate_station, "duplicate_sigma",
                 "INVALID_FORMATION_ASSESSMENT")):
            with self.subTest(field=label):
                tampered = copy.deepcopy(assessment)
                mutate(tampered)
                with self.assertRaises(VirtualAContractError) as raised:
                    _build_portal_candidates(scene, assessment=tampered)
                self.assertEqual(raised.exception.reason, reason)

    def test_missing_piece_wrong_region_and_fake_hashes_are_rejected(self):
        scene = _portal_scene(overlap_half=0.3)
        template = _build_portal_candidates(scene).templates[0]
        segment_index, cover_index, segment, cover = next(
            (index, position, item, candidate)
            for index, item in enumerate(template.segments)
            for position, candidate in enumerate(item.covers)
            if candidate.decomposition != "whole")
        self.assertEqual(cover.decomposition[:9], "diagonal_")

        # A decomposition with the wrong piece count / vertices is rejected
        # by the immutable template contract itself.
        bad_cover = replace(
            cover, decomposition="whole", pieces=(cover.pieces[0],))
        with self.assertRaises(VirtualAContractError) as raised:
            replace(template, segments=tuple(
                replace(item, covers=tuple(
                    bad_cover if position == cover_index else candidate
                    for position, candidate in enumerate(item.covers)))
                if index == segment_index else item
                for index, item in enumerate(template.segments)))
        self.assertEqual(
            raised.exception.reason, "INVALID_FORMATION_CORRIDOR_TEMPLATE")

        # A gap/overlap in the sigma partition is rejected by construction.
        with self.assertRaises(VirtualAContractError) as raised:
            replace(template, segments=(
                replace(segment, sigma_end=segment.sigma_end - 1e-6),
            ) + template.segments[1:])
        self.assertEqual(
            raised.exception.reason, "INVALID_FORMATION_CORRIDOR_TEMPLATE")

        # An out-of-chain region passes local piece construction but must be
        # rejected by the identity-aware validator.
        out_of_chain = replace(
            segment,
            covers=tuple(
                replace(
                    item,
                    pieces=tuple(
                        replace(piece, region_index=len(
                            scene["chain"].regions) + 3)
                        for piece in item.pieces))
                for item in segment.covers))
        tampered = replace(
            template,
            segments=(out_of_chain,) + template.segments[1:])
        with self.assertRaises(VirtualAContractError) as raised:
            validate_formation_template(
                tampered, scene["request"], scene["chain"],
                allocation=scene["allocation"], guide=scene["guide"],
                planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(
            raised.exception.reason, "INVALID_FORMATION_CORRIDOR_TEMPLATE")

        fake = replace(template, guide_sha256="0" * 64)
        with self.assertRaises(VirtualAContractError) as raised:
            validate_formation_template(
                fake, scene["request"], scene["chain"],
                allocation=scene["allocation"], guide=scene["guide"],
                planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(
            raised.exception.reason,
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH")


# ---------------------------------------------------------------------------
# Step 3: independent finite-sample audit and at most one targeted cut
# ---------------------------------------------------------------------------


def _step3_scene(y_half=1.1, psi0=0.0, kappa=0.0, max_seconds=20.0):
    """One-region scene whose walls can be tighter than the OBB footprint."""
    request = _request(psi0, kappa)
    end = (S_LENGTH * math.cos(psi0), S_LENGTH * math.sin(psi0))
    chain = CorridorChain(
        regions=(_box(-50.0, -float(y_half), 50.0, float(y_half),
                      (0.0, 0.0), end),),
        route_source="step3_fixture", grid_sha256=GRID_SHA)
    guide = make_corridor_guide(chain, (0.0, 0.0), end)
    allocation = allocate_corridors(chain, guide, INTERVALS)
    config = replace_config_scope(
        planner_config(
            guidance_mode="corridor",
            corridor_quadrature_points=9, solver="ipopt", max_initials=1),
        FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
        max_seconds=float(max_seconds))
    template = single_region_template(
        request, chain, allocation, guide,
        planning_context_sha256=CONTEXT_SHA, region_indices=0,
        margin_m=MARGIN_M, margin_source="step3_fixture")
    constraint = build_formation_corridor_constraint(
        template, request, chain, config, allocation=allocation,
        guide=guide, planning_context_sha256=CONTEXT_SHA)
    return {
        "request": request,
        "chain": chain,
        "guide": guide,
        "allocation": allocation,
        "config": config,
        "template": template,
        "constraint": constraint,
        "path": VirtualAPath(
            request.start_qf, _constant_profile(kappa), request.model),
    }


def _fake_ipopt_result(vector, *, success=True, timed_out=False):
    status = "Solve_Succeeded" if success else (
        "Maximum_WallTime_Exceeded" if timed_out
        else "Infeasible_Problem_Detected")
    return {
        "solver": "ipopt",
        "z": np.asarray(vector, dtype=float),
        "objective": 0.0,
        "solver_status": status,
        "success": bool(success),
        "iterations": 1,
        "detail": status,
        "wall_time_s": 0.01,
        "soft_deadline_triggered": bool(timed_out),
        "deadline_trigger_stage": (
            "ipopt_max_wall_time" if timed_out else ""),
        "remaining_budget_s": 20.0,
    }


def _audit_payload(template, status, *, reason="", cut_sigma=None,
                   witness=None):
    return {
        "schema": "virtual_a_formation_sample_audit_v1",
        "role": "independent_finite_sample_audit_not_a_certificate",
        "template_id": template.template_id,
        "status": status,
        "reason": reason,
        "sample_count": 10,
        "planned_sample_count": 10,
        "spacing_sigma": 0.005,
        "max_samples": 2048,
        "path_length_m": S_LENGTH,
        "definite_margin_m": 1e-9,
        "max_residual_m": (
            None if witness is None else witness.get("residual_m")),
        "max_evidence_m": (
            None if witness is None else witness.get("evidence_m")),
        "witness": witness,
        "cut_sigma": cut_sigma,
        "is_continuous_certificate": False,
        "certificate_note": (
            "finite independent samples only; Step 4 must certify intervals"),
    }


def _targeted_samples(constraint):
    formation = getattr(constraint, "formation", None)
    if formation is None:
        return []
    return [
        sample for group in formation.groups for sample in group
        if str(sample.source) == "targeted_cut"]


class FormationSampleAuditStep3Tests(unittest.TestCase):
    """Step 3: samples direct at most one cut; no blind or extra points."""

    def test_clear_audit_on_accepted_union_candidate_preserves_19_vars(self):
        scene = _scene()
        request = scene["request"]
        result = solve_open_path_guided(
            request, scene["config"],
            route_guides=(scene["guide"],),
            corridor_chain=scene["chain"],
            corridor_allocation=scene["allocation"],
            formation_template=scene["template"],
            formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        audit = result.formation_sample_audit
        self.assertEqual(audit.get("schema"),
                         "virtual_a_formation_sample_audit_v1")
        self.assertEqual(audit.get("status"),
                         FORMATION_SAMPLE_AUDIT_CLEAR)
        self.assertEqual(audit.get("template_id"),
                         scene["template"].template_id)
        self.assertGreater(audit.get("sample_count", 0), 0)
        self.assertGreater(audit.get("planned_sample_count", 0), 0)
        self.assertLessEqual(audit.get("max_residual_m"), 0.0)
        self.assertFalse(audit.get("is_continuous_certificate"))
        refinement = result.formation_refinement
        self.assertFalse(refinement["used"])
        self.assertEqual(refinement["outcome"], "NOT_NEEDED")
        self.assertEqual(refinement["cuts_sigma"], [])
        self.assertEqual(refinement["attempts"], 0)
        self.assertEqual(refinement["initial_obb_constraint_count"],
                         scene["constraint"].scalar_row_count)
        self.assertEqual(refinement["final_obb_constraint_count"],
                         scene["constraint"].scalar_row_count)
        self.assertIsNotNone(result.formation_obb_max_residual_m)
        self.assertLessEqual(result.formation_obb_max_residual_m, 1e-8)
        accepted = result.attempts[-1]
        self.assertEqual(accepted.status, "SOLVER_SUCCESS_AUDITED")
        self.assertEqual(accepted.initial.S, S_LENGTH)
        self.assertEqual(len(accepted.initial.kappas), INTERVALS + 1)
        self.assertEqual(len(accepted.initial.etas), INTERVALS + 1)
        self.assertEqual(
            len(accepted.initial.kappas) - 1, INTERVALS)
        self.assertEqual(
            1 + len(accepted.initial.kappas)
            + len(accepted.initial.etas), 2 * INTERVALS + 3)
        self.assertEqual(
            result.as_dict()["formation_sample_audit"]["status"],
            FORMATION_SAMPLE_AUDIT_CLEAR)

    def test_direct_audit_reports_definite_corner_violation(self):
        scene = _step3_scene(y_half=1.1)
        audit = audit_formation_template_samples(
            scene["path"], scene["template"], scene["chain"])
        self.assertEqual(audit["status"],
                         FORMATION_SAMPLE_AUDIT_VIOLATED)
        self.assertFalse(audit["is_continuous_certificate"])
        self.assertIsNotNone(audit["cut_sigma"])
        self.assertGreater(audit["cut_sigma"], 0.0)
        self.assertLess(audit["cut_sigma"], 1.0)
        witness = audit["witness"]
        self.assertIn(witness["vehicle_id"], (2, 3))
        self.assertGreater(witness["residual_m"], 0.14)
        self.assertGreater(witness["evidence_m"], 0.14)
        self.assertGreater(audit["max_residual_m"], 0.14)

        # The straight F reference path is inside the old corridor rows, while
        # the same vector violates the appended complete-OBB rows: this is the
        # F-feasible-but-vehicle-corner-out case, not an F failure.
        request = scene["request"]
        config = scene["config"]
        packet = _search_packet(request, config)
        constraint = _build_corridor_constraint(
            scene["chain"], scene["allocation"], request, config,
            formation_template=scene["template"],
            planning_context_sha256=CONTEXT_SHA,
            formation_guide=scene["guide"])
        _equality_end, signed_end, corridor_end = \
            _constraint_value_offsets(request, config, packet, constraint)
        values = _numeric_constraint_vector(
            _z(0.0), request, config, packet,
            int(config.collocation_points),
            corridor_constraint=constraint)
        formation_count = constraint.formation.scalar_row_count
        f_rows = values[signed_end:corridor_end - formation_count]
        obb_rows = values[corridor_end - formation_count:corridor_end]
        self.assertEqual(obb_rows.size, formation_count)
        self.assertLessEqual(float(np.max(f_rows)), 1e-9)
        self.assertGreater(float(np.max(obb_rows)), 0.14)

    def test_e04_offline_obb_rows_are_real_and_reject_f_only_feasible(self):
        """Offline E04 evidence: compiled OBB rows are real and rejecting.

        The live E04 fails closed in the Guide assessment map check before
        any NLP exists (``formation_obb_constraint_count`` is null there), so
        it cannot prove that the OBB rows themselves reject a path.  This
        test is the separate solver-path witness: the same pose that passes
        every historical F/corridor row has real, positive complete-OBB
        residuals, the returned NLP constraint carries a positive real OBB
        row count, and no candidate is accepted from that solve.
        """
        scene = _step3_scene(y_half=1.1, max_seconds=3.0)
        request = scene["request"]
        config = scene["config"]
        packet = _search_packet(request, config)
        constraint = _build_corridor_constraint(
            scene["chain"], scene["allocation"], request, config,
            formation_template=scene["template"],
            planning_context_sha256=CONTEXT_SHA,
            formation_guide=scene["guide"])
        # Real compiled rows: one positive scalar count per vehicle family,
        # not a polygon or a marker count.
        self.assertGreater(constraint.formation_obb_constraint_count, 0)
        self.assertEqual(constraint.formation_obb_constraint_count,
                         int(constraint.formation.scalar_row_count))
        vehicles = dict(constraint.formation.rows_by_vehicle)
        self.assertEqual(set(vehicles), {1, 2, 3})
        self.assertTrue(all(count > 0 for count in vehicles.values()),
                        vehicles)
        _equality_end, signed_end, corridor_end = \
            _constraint_value_offsets(request, config, packet, constraint)
        values = _numeric_constraint_vector(
            _z(0.0), request, config, packet,
            int(config.collocation_points),
            corridor_constraint=constraint)
        formation_count = constraint.formation.scalar_row_count
        f_rows = values[signed_end:corridor_end - formation_count]
        obb_rows = values[corridor_end - formation_count:corridor_end]
        self.assertEqual(int(obb_rows.size), formation_count)
        self.assertLessEqual(float(np.max(f_rows)), 1e-9)
        self.assertGreater(float(np.max(obb_rows)), 0.14)

        result = solve_open_path_guided(
            request, config,
            route_guides=(scene["guide"],),
            corridor_chain=scene["chain"],
            corridor_allocation=scene["allocation"],
            formation_template=scene["template"],
            formation_planning_context_sha256=CONTEXT_SHA)
        # The solver path really entered the NLP with a non-null OBB row
        # count, and the F-feasible/OBB-violating candidate was never
        # accepted.
        self.assertIsNotNone(result.formation_obb_constraint_count)
        self.assertGreater(result.formation_obb_constraint_count, 0)
        self.assertNotEqual(result.status, "GEOMETRY_FEASIBLE",
                            result.as_dict())
        self.assertIsNone(result.path)
        self.assertFalse(any(
            attempt.status == "SOLVER_SUCCESS_AUDITED"
            for attempt in result.attempts))
        if result.formation_obb_max_residual_m is not None:
            self.assertGreater(result.formation_obb_max_residual_m, 0.14)

    def test_f_feasible_but_obb_infeasible_solve_is_never_accepted(self):
        scene = _step3_scene(y_half=1.1, max_seconds=3.0)
        request = scene["request"]
        result = solve_open_path_guided(
            request, scene["config"],
            route_guides=(scene["guide"],),
            corridor_chain=scene["chain"],
            corridor_allocation=scene["allocation"],
            formation_template=scene["template"],
            formation_planning_context_sha256=CONTEXT_SHA)
        self.assertNotEqual(result.status, "GEOMETRY_FEASIBLE",
                            result.as_dict())
        self.assertIsNone(result.path)
        self.assertFalse(any(
            attempt.status in ("SOLVER_SUCCESS_AUDITED",)
            for attempt in result.attempts))
        if result.formation_obb_max_residual_m is not None:
            self.assertGreater(result.formation_obb_max_residual_m, 0.14)
        # At least one real solver attempt or a closed numerical failure was
        # recorded; no candidate was invented from the infeasible template.
        self.assertTrue(result.attempts)
        self.assertIn(result.status, (
            "NO_ACCEPTED_CANDIDATE", "NUMERICAL_FAILURE",
            "BUDGET_EXHAUSTED"))

    def test_positive_residual_inside_error_budget_is_unconfirmed(self):
        # Walls placed 5e-10 m inside the exact corner: the residual is
        # positive but far below the 1e-9 m definite threshold, so the audit
        # must report UNCONFIRMED and provide no cut.
        y_half = 1.25 + MARGIN_M - 5e-10
        scene = _step3_scene(y_half=y_half)
        audit = audit_formation_template_samples(
            scene["path"], scene["template"], scene["chain"])
        self.assertEqual(audit["status"],
                         FORMATION_SAMPLE_AUDIT_UNCONFIRMED)
        self.assertIsNone(audit["cut_sigma"])
        self.assertGreater(audit["max_residual_m"], 0.0)
        self.assertLess(audit["max_residual_m"], 1e-9)

    def test_unconfirmed_audit_adds_no_cut_and_no_extra_solver(self):
        scene = _scene()
        calls = []

        def fake_solve(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            calls.append((guess, corridor_constraint))
            return _fake_ipopt_result(_z(0.0))

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solve), \
                mock.patch.object(
                    optimizer_module, "_formation_sample_audit",
                    side_effect=lambda path, template, chain: _audit_payload(
                        template, FORMATION_SAMPLE_AUDIT_UNCONFIRMED,
                        reason="positive residual inside budget")):
            result = solve_open_path_guided(
                scene["request"], scene["config"],
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(len(calls), 1)
        self.assertEqual(_targeted_samples(calls[0][1]), [])
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason,
                         "FORMATION_CORRIDOR_AUDIT_UNCONFIRMED")
        self.assertEqual(len(result.attempts), 1)
        self.assertEqual(result.attempts[0].status,
                         "SOLVER_SUCCESS_FORMATION_AUDIT_UNCONFIRMED")
        self.assertFalse(result.formation_refinement["used"])
        self.assertEqual(result.formation_refinement["attempts"], 0)
        self.assertEqual(result.formation_refinement["outcome"],
                         "UNCONFIRMED_NO_CUT")
        self.assertEqual(result.formation_refinement["cuts_sigma"], [])

    def test_one_definite_cut_reenters_actual_g_rows(self):
        scene = _scene()
        request = scene["request"]
        config = scene["config"]
        calls = []
        witness = {
            "sigma": 0.3, "s_m": 0.3 * S_LENGTH,
            "segment_index": 0, "vehicle_id": 2, "decomposition": "whole",
            "body_piece_index": 0, "region_index": 0, "halfspace_id": 3,
            "vertex_id": 1, "point_x": 1.0, "point_y": 2.0,
            "residual_m": 1.0, "error_budget_m": 0.0,
            "evidence_m": 1.0,
        }
        audits = iter([
            _audit_payload(
                scene["template"], FORMATION_SAMPLE_AUDIT_VIOLATED,
                reason="definite OBB violation",
                cut_sigma=0.3, witness=witness),
            _audit_payload(
                scene["template"], FORMATION_SAMPLE_AUDIT_CLEAR,
                reason=""),
        ])

        def fake_audit(path, template, chain):
            return next(audits)

        def fake_solve(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            calls.append((guess, corridor_constraint))
            return _fake_ipopt_result(_z(0.0))

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solve), \
                mock.patch.object(
                    optimizer_module, "_formation_sample_audit",
                    side_effect=fake_audit):
            result = solve_open_path_guided(
                request, config,
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(len(calls), 2)
        initial_constraint = calls[0][1]
        refined_constraint = calls[1][1]
        self.assertEqual(_targeted_samples(initial_constraint), [])
        targeted = _targeted_samples(refined_constraint)
        self.assertTrue(targeted)
        for sample in targeted:
            self.assertAlmostEqual(sample.sigma, 0.3, places=12)
        initial_formation = initial_constraint.formation
        refined_formation = refined_constraint.formation
        added_samples = (
            refined_formation.sample_count
            - initial_formation.sample_count)
        added_rows = (
            refined_formation.scalar_row_count
            - initial_formation.scalar_row_count)
        self.assertEqual(added_samples, 3)
        self.assertGreater(added_rows, 0)
        self.assertEqual(
            float(len(targeted)), float(added_samples))

        # The refined constraint is rebuilt from the real row plan: the row
        # count is read from the appended scalar rows, and the same rows are
        # present in both the numeric vector and CasADi g in the same order.
        packet = _search_packet(request, config)
        _eq, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, refined_constraint)
        f_corridor_rows = sum(
            len(limits) for group in refined_constraint.groups
            for _q, _rows, limits in group)
        # Under union scope the fixed allocation F-only rows are not
        # compiled; the template F rows are the F assignment.
        self.assertEqual(f_corridor_rows, 0)
        self.assertEqual(
            corridor_end - signed_end,
            refined_formation.formation_reference_constraint_count
            + refined_formation.scalar_row_count)
        vector = _z(0.0)
        numeric = _numeric_constraint_vector(
            vector, request, config, packet,
            int(config.collocation_points),
            corridor_constraint=refined_constraint)
        self.assertEqual(numeric.size, corridor_end)
        if casadi is not None:
            nlp, _built = _casadi_nlp(
                request, config, packet, refined_constraint)
            g_function = casadi.Function(
                "g", [nlp["x"]], [nlp["g"]])
            symbolic = np.asarray(
                g_function(vector), dtype=float).ravel()
            self.assertEqual(symbolic.size, corridor_end)
            np.testing.assert_allclose(
                symbolic, numeric, atol=1e-8, rtol=0.0)

        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertEqual(result.formation_obb_constraint_count,
                         refined_formation.scalar_row_count)
        self.assertEqual(result.formation_refinement["used"], True)
        self.assertEqual(result.formation_refinement["attempts"], 1)
        self.assertEqual(result.formation_refinement["cuts_sigma"],
                         [0.3])
        self.assertEqual(
            result.formation_refinement["initial_obb_constraint_count"],
            initial_formation.scalar_row_count)
        self.assertEqual(
            result.formation_refinement["final_obb_constraint_count"],
            refined_formation.scalar_row_count)
        self.assertEqual(
            result.formation_refinement["outcome"], "ACCEPTED")
        self.assertEqual(
            [attempt.status for attempt in result.attempts],
            ["SOLVER_SUCCESS_FORMATION_REFINEMENT",
             "SOLVER_SUCCESS_AUDITED"])
        self.assertEqual(len(calls[0][0].vector), 2 * INTERVALS + 3)
        self.assertEqual(len(calls[1][0].vector), 2 * INTERVALS + 3)
        self.assertEqual(calls[1][0].source, "formation_refinement")
        self.assertTrue(calls[1][0].label.endswith("|targeted_cut"))

    def test_second_definite_violation_stops_without_a_third_solver(self):
        scene = _scene()
        calls = []
        cuts = iter([0.3, 0.4375])

        def fake_audit(path, template, chain):
            cut = next(cuts)
            return _audit_payload(
                template, FORMATION_SAMPLE_AUDIT_VIOLATED,
                reason="definite OBB violation", cut_sigma=cut,
                witness={"sigma": cut, "vehicle_id": 2,
                         "residual_m": 1.0, "evidence_m": 1.0})

        def fake_solve(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            calls.append((guess, corridor_constraint))
            return _fake_ipopt_result(_z(0.0))

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solve), \
                mock.patch.object(
                    optimizer_module, "_formation_sample_audit",
                    side_effect=fake_audit):
            result = solve_open_path_guided(
                scene["request"], scene["config"],
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(len(calls), 2)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason,
                         "FORMATION_CORRIDOR_AUDIT_VIOLATION")
        self.assertEqual(result.formation_refinement["used"], True)
        self.assertEqual(result.formation_refinement["cuts_sigma"],
                         [0.3])
        self.assertEqual(
            [attempt.status for attempt in result.attempts],
            ["SOLVER_SUCCESS_FORMATION_REFINEMENT",
             "SOLVER_SUCCESS_FORMATION_AUDIT_REJECTED"])

    def test_refinement_timeout_starts_no_extra_solver(self):
        scene = _scene()
        calls = []

        def fake_audit(path, template, chain):
            return _audit_payload(
                template, FORMATION_SAMPLE_AUDIT_VIOLATED,
                reason="definite OBB violation", cut_sigma=0.3,
                witness={"sigma": 0.3, "vehicle_id": 2,
                         "residual_m": 1.0, "evidence_m": 1.0})

        def fake_solve(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            calls.append((guess, corridor_constraint))
            if len(calls) == 1:
                return _fake_ipopt_result(_z(0.0))
            return _fake_ipopt_result(
                _z(0.0), success=False, timed_out=True)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solve), \
                mock.patch.object(
                    optimizer_module, "_formation_sample_audit",
                    side_effect=fake_audit):
            result = solve_open_path_guided(
                scene["request"], scene["config"],
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(len(calls), 2)
        self.assertEqual(result.status, "BUDGET_EXHAUSTED")
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual(result.deadline_summary["trigger_stage"],
                         "formation_refinement_timeout")
        self.assertEqual(result.formation_refinement["outcome"], "TIMEOUT")
        self.assertEqual(result.attempts[-1].status, "SOLVER_TIMEOUT")

    def test_refinement_solver_failure_is_recorded_and_not_accepted(self):
        scene = _scene()
        calls = []

        def fake_audit(path, template, chain):
            return _audit_payload(
                template, FORMATION_SAMPLE_AUDIT_VIOLATED,
                reason="definite OBB violation", cut_sigma=0.3,
                witness={"sigma": 0.3, "vehicle_id": 2,
                         "residual_m": 1.0, "evidence_m": 1.0})

        def fake_solve(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            calls.append((guess, corridor_constraint))
            if len(calls) == 1:
                return _fake_ipopt_result(_z(0.0))
            return _fake_ipopt_result(_z(0.0), success=False)

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solve), \
                mock.patch.object(
                    optimizer_module, "_formation_sample_audit",
                    side_effect=fake_audit):
            result = solve_open_path_guided(
                scene["request"], scene["config"],
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(len(calls), 2)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(
            result.reason,
            "FORMATION_CORRIDOR_REFINEMENT_SOLVER_FAILED")
        self.assertEqual(result.formation_refinement["outcome"],
                         "REJECTED")
        self.assertEqual(result.attempts[-1].status,
                         "SOLVER_SUCCESS_FORMATION_AUDIT_REJECTED")

    def test_definite_cut_at_compiled_station_adds_no_cut(self):
        scene = _scene()
        calls = []

        def fake_audit(path, template, chain):
            return _audit_payload(
                template, FORMATION_SAMPLE_AUDIT_VIOLATED,
                reason="definite OBB violation", cut_sigma=0.3125,
                witness={"sigma": 0.3125, "vehicle_id": 2,
                         "residual_m": 1.0, "evidence_m": 1.0})

        def fake_solve(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            calls.append(corridor_constraint)
            return _fake_ipopt_result(_z(0.0))

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solve), \
                mock.patch.object(
                    optimizer_module, "_formation_sample_audit",
                    side_effect=fake_audit):
            result = solve_open_path_guided(
                scene["request"], scene["config"],
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        # 0.3125 is the Gauss node at (j=2, node=0.5): a definite witness on
        # an already compiled station cannot spend the single refinement.
        self.assertEqual(len(calls), 1)
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason,
                         "FORMATION_CORRIDOR_AUDIT_VIOLATION")
        self.assertFalse(result.formation_refinement["used"])
        self.assertEqual(result.formation_refinement["attempts"], 0)
        self.assertEqual(result.formation_refinement["outcome"],
                         "CUT_ALREADY_PRESENT")
        self.assertEqual(result.attempts[-1].status,
                         "SOLVER_SUCCESS_FORMATION_AUDIT_REJECTED")

    def test_scipy_candidate_refines_with_scipy_same_family(self):
        scene = _scene()
        config = replace(scene["config"], solver="scipy")
        calls = []
        audits = iter([
            _audit_payload(
                scene["template"], FORMATION_SAMPLE_AUDIT_VIOLATED,
                reason="definite OBB violation", cut_sigma=0.3,
                witness={"sigma": 0.3, "vehicle_id": 2,
                         "residual_m": 1.0, "evidence_m": 1.0}),
            _audit_payload(
                scene["template"], FORMATION_SAMPLE_AUDIT_CLEAR),
        ])

        def fake_audit(path, template, chain):
            return next(audits)

        def fake_slsqp(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            calls.append((guess, corridor_constraint))
            result = _fake_ipopt_result(_z(0.0))
            result["solver"] = "scipy_slsqp"
            result["solver_status"] = "SLSQP_SOLVED"
            return result

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=AssertionError(
                    "IPOPT must not run for a SciPy candidate")), \
                mock.patch.object(
                    optimizer_module, "_solve_with_scipy",
                    side_effect=fake_slsqp), \
                mock.patch.object(
                    optimizer_module, "_formation_sample_audit",
                    side_effect=fake_audit):
            result = solve_open_path_guided(
                scene["request"], config,
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(len(calls), 2)
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertEqual(result.solver_used, "scipy_slsqp")
        self.assertTrue(result.formation_refinement["used"])
        self.assertEqual(result.formation_refinement["solver"],
                         "scipy_slsqp")
        self.assertEqual(result.formation_refinement["outcome"],
                         "ACCEPTED")


def _slanted_projection_max(path, frames, normal_x, normal_y, s_m):
    """Largest F/OBB projection on one normal, computed from the map state."""
    state = path.state_at_s(s_m)
    values = [normal_x * state.qf.x + normal_y * state.qf.y]
    for vehicle in state.vehicles:
        frame = frames[vehicle.vehicle_id]
        cosine = math.cos(vehicle.theta)
        sine = math.sin(vehicle.theta)
        for local_x, local_y in frame.local_corners:
            values.append(
                normal_x * (vehicle.x + cosine * local_x - sine * local_y)
                + normal_y * (vehicle.y + sine * local_x + cosine * local_y))
    return max(values)


def _slanted_wall_scene(kappa=0.5, angle_deg=51.0, fine_steps=2400,
                        clearance_m=None):
    """One slanted wall over a curved path.

    With ``clearance_m`` the wall is that far outside the continuous OBB
    maximum (a positive certificate).  Without it the wall is placed midway
    between the collocation maximum and the continuous maximum: every finite
    row station passes while the real OBB crosses the wall between stations.
    """
    request = _request(0.0, kappa)
    path = VirtualAPath(
        request.start_qf, _constant_profile(kappa), request.model)
    frames = {frame.vehicle_id: frame for frame in vehicle_frames(request)}
    angle = math.radians(float(angle_deg))
    normal_x, normal_y = math.cos(angle), math.sin(angle)
    end_state = path.state_at_s(path.length_m)
    seed_end = (end_state.qf.x, end_state.qf.y)

    def build(wall):
        region = ConvexCorridor(
            seed_start=(request.start_qf.x, request.start_qf.y),
            seed_end=seed_end,
            A=((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0),
               (normal_x, normal_y)),
            b=(50.0, 50.0, 50.0, float(wall)),
            vertices=(
                (-50.0, -50.0), (50.0, -50.0),
                (50.0, (wall - 50.0 * normal_x) / normal_y),
                (-50.0, (wall + 50.0 * normal_x) / normal_y)))
        chain = CorridorChain(
            regions=(region,), route_source="slanted_wall_fixture",
            grid_sha256=GRID_SHA)
        guide = make_corridor_guide(
            chain, (request.start_qf.x, request.start_qf.y), seed_end)
        allocation = allocate_corridors(chain, guide, INTERVALS)
        config = planner_config(
            guidance_mode="corridor",
            formation_corridor_scope=(
                FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1),
            corridor_quadrature_points=9, solver="ipopt",
            max_initials=1, max_seconds=30.0)
        template = single_region_template(
            request, chain, allocation, guide,
            planning_context_sha256=CONTEXT_SHA, region_indices=0,
            margin_m=MARGIN_M, margin_source="slanted_wall_fixture")
        constraint = build_formation_corridor_constraint(
            template, request, chain, config, allocation=allocation,
            guide=guide, planning_context_sha256=CONTEXT_SHA)
        return {"chain": chain, "guide": guide, "allocation": allocation,
                "config": config, "template": template,
                "constraint": constraint}

    # A loose provisional wall only supplies the real collocation stations.
    provisional = build(50.0)
    station_max = max(
        _slanted_projection_max(
            path, frames, normal_x, normal_y,
            float(sample.sigma) * path.length_m)
        for group in provisional["constraint"].groups
        for sample in group)
    sampled = int(fine_steps)
    fine_max = max(
        _slanted_projection_max(
            path, frames, normal_x, normal_y,
            path.length_m * index / float(sampled))
        for index in range(sampled + 1))
    wall = (0.5 * (station_max + fine_max) if clearance_m is None
            else fine_max + float(clearance_m))
    return dict(
        request=request, path=path, wall_m=wall,
        station_max_m=station_max, fine_max_m=fine_max, **build(wall))


def _side_wall_scene(kappa=-0.5, deficit_m=None, clearance_m=None,
                     fine_steps=2400):
    """A turning path inside one convex box with a vertical side wall.

    The wall is the ``x <= wall`` halfspace.  With negative curvature the
    OBB maximum projection onto this wall peaks *inside* a 0.75 m cut
    interval: the heading at the interval end is far past the peak, which is
    exactly the state the old scalar-width heading increment failed to
    enclose.  With ``clearance_m`` the wall sits that far outside the dense
    continuous maximum (a positive certificate); without it the wall sits
    midway between the collocation maximum and the continuous maximum, so
    every finite row station passes while the real path crosses the wall.
    """
    request = _request(0.0, kappa)
    path = VirtualAPath(
        request.start_qf, _constant_profile(kappa), request.model)
    frames = {frame.vehicle_id: frame for frame in vehicle_frames(request)}
    end_state = path.state_at_s(path.length_m)
    seed_end = (end_state.qf.x, end_state.qf.y)

    def build(wall):
        region = ConvexCorridor(
            seed_start=(request.start_qf.x, request.start_qf.y),
            seed_end=seed_end,
            A=((-1.0, 0.0), (0.0, -1.0), (0.0, 1.0), (1.0, 0.0)),
            b=(50.0, 50.0, 50.0, float(wall)),
            vertices=(
                (-50.0, -50.0), (float(wall), -50.0),
                (float(wall), 50.0), (-50.0, 50.0)))
        chain = CorridorChain(
            regions=(region,), route_source="side_wall_fixture",
            grid_sha256=GRID_SHA)
        guide = make_corridor_guide(
            chain, (request.start_qf.x, request.start_qf.y), seed_end)
        allocation = allocate_corridors(chain, guide, INTERVALS)
        config = planner_config(
            guidance_mode="corridor",
            formation_corridor_scope=(
                FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1),
            corridor_quadrature_points=9, solver="ipopt",
            max_initials=1, max_seconds=30.0)
        template = single_region_template(
            request, chain, allocation, guide,
            planning_context_sha256=CONTEXT_SHA, region_indices=0,
            margin_m=MARGIN_M, margin_source="side_wall_fixture")
        constraint = build_formation_corridor_constraint(
            template, request, chain, config, allocation=allocation,
            guide=guide, planning_context_sha256=CONTEXT_SHA)
        return {"chain": chain, "guide": guide, "allocation": allocation,
                "config": config, "template": template,
                "constraint": constraint}

    # A loose provisional wall only supplies the real collocation stations.
    provisional = build(50.0)
    station_max = max(
        _slanted_projection_max(
            path, frames, 1.0, 0.0, float(sample.sigma) * path.length_m)
        for group in provisional["constraint"].groups
        for sample in group)
    sampled = int(fine_steps)
    fine_max = max(
        _slanted_projection_max(
            path, frames, 1.0, 0.0, path.length_m * index / float(sampled))
        for index in range(sampled + 1))
    if deficit_m is not None:
        wall = fine_max - float(deficit_m)
    elif clearance_m is not None:
        wall = fine_max + float(clearance_m)
    else:
        wall = 0.5 * (station_max + fine_max)
    return dict(
        request=request, path=path, wall_m=wall,
        station_max_m=station_max, fine_max_m=fine_max, **build(wall))


def _certificate_for(scene, *, template=None, path=None, **limits):
    return certify_formation_path(
        scene["path"] if path is None else path,
        scene["request"], scene["chain"],
        scene["template"] if template is None else template,
        allocation=scene["allocation"], guide=scene["guide"],
        planning_context_sha256=CONTEXT_SHA, config=scene["config"],
        **limits)


class FormationContinuousCertificateStep4Tests(unittest.TestCase):
    """Step 4: only VERIFIED identity-matching continuous certificates gate."""

    def test_wide_scene_verified_certificate_is_deterministic(self):
        scene = _scene()
        first = _certificate_for(scene)
        second = _certificate_for(scene)
        self.assertEqual(first["schema"], FORMATION_CERTIFICATE_SCHEMA)
        self.assertEqual(first["status"], FORMATION_CERTIFICATE_VERIFIED)
        self.assertEqual(first["reason"], "")
        self.assertTrue(first["is_continuous_certificate"])
        self.assertFalse(first["finite_sample_audit_is_certificate"])
        self.assertGreater(first["intervals_verified"], 0)
        self.assertGreaterEqual(
            first["intervals_checked"], first["intervals_verified"])
        self.assertEqual(first["unclosed_interval_count"], 0)
        self.assertLessEqual(first["max_outward_residual_m"], 0.0)
        self.assertIsNone(first["witness"])
        self.assertFalse(first["timed_out"])
        self.assertEqual(first["certificate_sha256"],
                         second["certificate_sha256"])
        self.assertEqual(
            {item["vehicle_id"] for item in first["vehicle_coverage"]},
            {1, 2, 3})
        cuts = first["cut_checks"]
        self.assertEqual(cuts[0]["s_m"], 0.0)
        self.assertEqual(cuts[0]["side"], "right")
        self.assertEqual(cuts[-1]["s_m"], S_LENGTH)
        self.assertEqual(cuts[-1]["side"], "left")
        self.assertTrue(all(
            check["status"] == "CLOSED" for check in cuts))
        sides_by_station = {}
        for check in cuts:
            sides_by_station.setdefault(
                round(check["s_m"], 9), set()).add(check["side"])
        for station, sides in sides_by_station.items():
            if station in (0.0, S_LENGTH):
                continue
            self.assertEqual(sides, {"left", "right"})
        comparison = first["f_only_audit"]
        self.assertEqual(
            comparison["certificate_scope"],
            "f_only_comparison_not_the_three_obb_gate")
        self.assertTrue(comparison["passed"])
        self.assertLessEqual(comparison["max_bound_residual_m"], 0.0)
        verification = verify_formation_certificate(
            first, scene["path"], scene["request"], scene["chain"],
            scene["template"], allocation=scene["allocation"],
            guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertTrue(verification["passed"], verification)
        self.assertEqual(verification["certificate_sha256"],
                         first["certificate_sha256"])

    def test_collocation_safe_but_continuous_corner_violation(self):
        scene = _slanted_wall_scene()
        self.assertLess(scene["station_max_m"], scene["wall_m"])
        self.assertGreater(scene["fine_max_m"], scene["wall_m"])
        certificate = _certificate_for(scene)
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_VIOLATED)
        self.assertEqual(certificate["reason"],
                         "DEFINITE_INTERVAL_VIOLATION")
        witness = certificate["witness"]
        self.assertIsNotNone(witness)
        self.assertEqual(witness["kind"], "obb")
        self.assertEqual(witness["vehicle_id"], 3)
        self.assertGreater(witness["sigma"], 0.0)
        self.assertLess(witness["sigma"], 1.0)
        self.assertGreater(witness["residual_m"], 0.0)
        self.assertGreater(witness["evidence_m"], 1e-9)
        # The old F-only comparison passes; it can never see a corner.
        self.assertTrue(certificate["f_only_audit"]["passed"])
        self.assertEqual(
            certificate["f_only_audit"]["reason"],
            "CORRIDOR_CONTAINMENT_VERIFIED")

        # Independent recomputation from the map state: the witness corner is
        # a real OBB corner outside the wall, not a certificate-internal toy.
        frames = {
            frame.vehicle_id: frame
            for frame in vehicle_frames(scene["request"])}
        state = scene["path"].state_at_s(witness["s_m"])
        vehicle = [
            item for item in state.vehicles
            if item.vehicle_id == witness["vehicle_id"]][0]
        frame = frames[vehicle.vehicle_id]
        cosine, sine = math.cos(vehicle.theta), math.sin(vehicle.theta)
        corners = [
            (vehicle.x + cosine * local_x - sine * local_y,
             vehicle.y + sine * local_x + cosine * local_y)
            for local_x, local_y in frame.local_corners]
        corner = min(
            corners,
            key=lambda point: math.hypot(
                point[0] - witness["point_x"],
                point[1] - witness["point_y"]))
        self.assertLess(
            math.hypot(
                corner[0] - witness["point_x"],
                corner[1] - witness["point_y"]), 1e-9)
        angle = math.radians(51.0)
        projection = (math.cos(angle) * corner[0]
                      + math.sin(angle) * corner[1])
        self.assertGreater(
            projection, scene["wall_m"] - MARGIN_M)

        verification = verify_formation_certificate(
            certificate, scene["path"], scene["request"], scene["chain"],
            scene["template"], allocation=scene["allocation"],
            guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertFalse(verification["passed"])
        self.assertEqual(verification["reason"],
                         "CERTIFICATE_NOT_VERIFIED")

    def test_template_and_hermite_cuts_are_two_sided(self):
        scene = _scene()
        base = scene["template"]
        left = replace(base.segments[0], sigma_end=0.4)
        right = replace(base.segments[0], sigma_start=0.4)
        split = replace(base, segments=(left, right))
        self.assertNotEqual(split.template_id, base.template_id)
        certificate = _certificate_for(scene, template=split)
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_VERIFIED)
        self.assertEqual(certificate["template_id"], split.template_id)
        sides_by_station = {}
        for check in certificate["cut_checks"]:
            self.assertEqual(check["status"], "CLOSED")
            sides_by_station.setdefault(
                round(check["s_m"], 9), set()).add(check["side"])
        self.assertEqual(sides_by_station[0.0], {"right"})
        self.assertEqual(sides_by_station[S_LENGTH], {"left"})
        self.assertEqual(sides_by_station[2.4], {"left", "right"})
        for boundary in (0.75, 1.5, 2.25, 3.0, 3.75, 4.5, 5.25):
            self.assertEqual(
                sides_by_station[boundary], {"left", "right"})
        self.assertEqual(
            certificate["intervals_verified"],
            len(sides_by_station) - 1)
        self.assertEqual(certificate["intervals_checked"],
                         certificate["intervals_verified"])
        verification = verify_formation_certificate(
            certificate, scene["path"], scene["request"], scene["chain"],
            split, allocation=scene["allocation"], guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertTrue(verification["passed"], verification)

    def test_depth_limit_nonclosure_is_unconfirmed(self):
        scene = _slanted_wall_scene(clearance_m=1e-3)
        certificate = _certificate_for(scene, max_depth=0)
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_UNCONFIRMED)
        self.assertEqual(certificate["reason"], "INTERVAL_NOT_CLOSED")
        self.assertIsNone(certificate["witness"])
        self.assertGreater(certificate["unclosed_interval_count"], 0)
        self.assertTrue(certificate["unclosed_intervals"])
        self.assertEqual(certificate["max_depth_reached"], 0)
        verification = verify_formation_certificate(
            certificate, scene["path"], scene["request"], scene["chain"],
            scene["template"], allocation=scene["allocation"],
            guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertFalse(verification["passed"])
        self.assertEqual(verification["reason"],
                         "CERTIFICATE_NOT_VERIFIED")

    def test_interval_and_time_limits_are_unconfirmed(self):
        scene = _slanted_wall_scene(clearance_m=1e-3)
        capped = _certificate_for(scene, max_intervals=1)
        self.assertEqual(capped["status"],
                         FORMATION_CERTIFICATE_UNCONFIRMED)
        self.assertEqual(capped["reason"], "INTERVAL_LIMIT")
        self.assertTrue(capped["interval_limit_reached"])
        self.assertEqual(capped["intervals_checked"], 1)
        self.assertIsNone(capped["witness"])
        expired = _certificate_for(scene, max_seconds=0.0)
        self.assertEqual(expired["status"],
                         FORMATION_CERTIFICATE_UNCONFIRMED)
        self.assertEqual(expired["reason"], "TIME_LIMIT")
        self.assertTrue(expired["timed_out"])
        self.assertEqual(expired["intervals_checked"], 0)
        self.assertIsNone(expired["witness"])

    def test_normal_branch_degeneration_is_unconfirmed(self):
        scene = _scene(kappa=1.0)
        certificate = _certificate_for(scene, max_depth=4)
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_UNCONFIRMED)
        self.assertEqual(certificate["reason"], "INTERVAL_NOT_CLOSED")
        self.assertIsNone(certificate["witness"])
        unclosed = certificate["unclosed_intervals"]
        self.assertTrue(unclosed)
        self.assertTrue(all(
            item["outcome"] == "degenerate" for item in unclosed))
        self.assertTrue(any(
            "NORMAL_BRANCH_VIOLATION" in item["reason"]
            for item in unclosed))
        self.assertIn("A=0.0", unclosed[0]["reason"])
        self.assertIn("vehicle_id=2", unclosed[0]["reason"])

    def test_nonfinite_interval_is_unconfirmed(self):
        scene = _scene()
        real_bounds = C2CurvatureProfile.interval_bounds

        def nan_bounds(self, s0, s1):
            record = dict(real_bounds(self, s0, s1))
            record["k"] = [float("nan"), float("nan")]
            return record

        with mock.patch.object(
                C2CurvatureProfile, "interval_bounds", nan_bounds):
            certificate = _certificate_for(scene)
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_UNCONFIRMED)
        self.assertEqual(certificate["reason"], "INTERVAL_NOT_CLOSED")
        self.assertIsNone(certificate["witness"])
        unclosed = certificate["unclosed_intervals"]
        self.assertTrue(unclosed)
        self.assertTrue(all(
            item["outcome"] == "nonfinite" for item in unclosed))
        self.assertIn("nonfinite kappa", unclosed[0]["reason"])

    def test_identity_and_payload_tamper_fail_closed(self):
        scene = _scene()
        certificate = _certificate_for(scene)
        request = scene["request"]
        chain = scene["chain"]
        template = scene["template"]
        allocation = scene["allocation"]
        guide = scene["guide"]

        def verify(path=None, req=None, current_chain=None,
                   current_template=None, context=CONTEXT_SHA,
                   certificate_value=None):
            return verify_formation_certificate(
                certificate if certificate_value is None
                else certificate_value,
                scene["path"] if path is None else path,
                request if req is None else req,
                chain if current_chain is None else current_chain,
                template if current_template is None else current_template,
                allocation=allocation, guide=guide,
                planning_context_sha256=context)

        self.assertTrue(verify()["passed"])
        self.assertEqual(
            verify()["certificate_sha256"],
            certificate["certificate_sha256"])
        other_path = VirtualAPath(
            request.start_qf, _constant_profile(0.2), request.model)
        result = verify(path=other_path)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_IDENTITY_MISMATCH")
        wrong_request = replace(
            request,
            goal_qf=FormationPose(
                request.goal_qf.x, request.goal_qf.y,
                request.goal_qf.psi + 0.05))
        result = verify(req=wrong_request)
        self.assertFalse(result["passed"])
        self.assertIn(
            result["reason"],
            ("CERTIFICATE_IDENTITY_UNAVAILABLE",
             "CERTIFICATE_IDENTITY_MISMATCH"))
        wrong_chain = CorridorChain(
            regions=chain.regions, route_source=chain.route_source,
            grid_sha256="1" * 64)
        result = verify(current_chain=wrong_chain)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_IDENTITY_UNAVAILABLE")
        result = verify(context="d" * 64)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_IDENTITY_UNAVAILABLE")
        wrong_template = replace(template, margin_source="tampered")
        result = verify(current_template=wrong_template)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_IDENTITY_MISMATCH")
        wrong_model = replace(
            request,
            model=FormationModel([
                VehicleOffset(1, 1.0, 0.0),
                VehicleOffset(2, 0.0, 1.1),
                VehicleOffset(3, 0.0, -1.1)]))
        result = verify(req=wrong_model)
        self.assertFalse(result["passed"])
        self.assertIn(
            result["reason"],
            ("CERTIFICATE_IDENTITY_UNAVAILABLE",
             "CERTIFICATE_IDENTITY_MISMATCH"))

        tampered_body = copy.deepcopy(certificate)
        tampered_body["max_outward_residual_m"] = 1.0
        result = verify(certificate_value=tampered_body)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_HASH_MISMATCH")
        tampered_status = copy.deepcopy(certificate)
        tampered_status["status"] = FORMATION_CERTIFICATE_UNCONFIRMED
        result = verify(certificate_value=tampered_status)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_HASH_MISMATCH")
        wrong_schema = copy.deepcopy(certificate)
        wrong_schema["schema"] = "virtual_a_formation_certificate.v0"
        result = verify(certificate_value=wrong_schema)
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_SCHEMA_MISMATCH")
        timing_only = copy.deepcopy(certificate)
        timing_only["elapsed_s"] = float(
            timing_only["elapsed_s"]) + 1.0
        result = verify(certificate_value=timing_only)
        self.assertTrue(result["passed"], result)
        self.assertFalse(
            verify_formation_certificate(
                None, scene["path"], request, chain, template,
                allocation=allocation, guide=guide,
                planning_context_sha256=CONTEXT_SHA)["passed"])

    def test_signed_malformed_payload_fails_closed(self):
        scene = _scene()
        certificate = _certificate_for(scene)

        def reseal(**changes):
            payload = copy.deepcopy(certificate)
            payload.pop("certificate_sha256")
            payload.update(changes)
            return seal_certificate(payload)

        def verify_payload(payload):
            return verify_formation_certificate(
                payload, scene["path"], scene["request"], scene["chain"],
                scene["template"], allocation=scene["allocation"],
                guide=scene["guide"],
                planning_context_sha256=CONTEXT_SHA)

        result = verify_payload(
            reseal(unclosed_interval_count="not-an-int"))
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_COUNTS_UNAVAILABLE")
        result = verify_payload(reseal(cut_checks="not-a-list"))
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_CUTS_MISSING")
        result = verify_payload(
            reseal(cut_checks=("not-a-mapping",)))
        self.assertFalse(result["passed"])
        self.assertEqual(result["reason"],
                         "CERTIFICATE_CUT_NOT_CLOSED")

    def test_portal_transition_cuts_are_two_sided(self):
        scene = _portal_scene(overlap_half=0.3)
        theta = PORTAL_THETA
        start = (-2.0 * math.cos(theta), -2.0 * math.sin(theta))
        goal = (2.0 * math.cos(theta), 2.0 * math.sin(theta))
        request = request_from_fixture(
            "STRAIGHT_LINE",
            start_qf=FormationPose(start[0], start[1], theta),
            start_headings={
                vehicle_id: theta
                for vehicle_id in scene["request"].start_headings},
            goal_qf=FormationPose(goal[0], goal[1], theta))
        allocation = allocate_corridors(
            scene["chain"], scene["guide"], INTERVALS)
        assessment = _assess_portal(
            scene["chain"], scene["guide"], request, scene["grid"],
            allocation, theta, theta)
        candidates = build_formation_template_candidates(
            request, scene["chain"], scene["guide"], allocation,
            assessment, scene["config"],
            planning_context_sha256=CONTEXT_SHA, margin_m=MARGIN_M,
            margin_source="task03_step4_portal_fixture")
        self.assertTrue(candidates.templates)
        template = candidates.templates[0]
        path = VirtualAPath(
            request.start_qf, _constant_profile(0.0, length=4.0),
            request.model)
        certificate = certify_formation_path(
            path, request, scene["chain"], template,
            allocation=allocation, guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA,
            config=scene["config"])
        # A straight fixture path does not follow the selected Portal
        # transition state; the certificate must reject it at the concrete
        # transition cut, and every Portal boundary must still be checked on
        # both sides.
        portal_boundaries = {
            round(float(segment.sigma_start) * 4.0, 9)
            for segment in template.segments[1:]}
        self.assertTrue(portal_boundaries)
        sides_by_station = {}
        for check in certificate["cut_checks"]:
            sides_by_station.setdefault(
                round(float(check["s_m"]), 9), set()).add(check["side"])
        for boundary in portal_boundaries:
            self.assertIn(boundary, sides_by_station)
            self.assertEqual(
                sides_by_station[boundary], {"left", "right"})
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_VIOLATED)
        self.assertEqual(certificate["reason"],
                         "DEFINITE_CUT_VIOLATION")
        witness = certificate["witness"]
        self.assertEqual(witness["kind"], "obb")
        self.assertEqual(witness["vehicle_id"], 2)
        self.assertEqual(witness["region_index"], 0)
        self.assertGreater(witness["evidence_m"], 0.0)
        verification = verify_formation_certificate(
            certificate, path, request, scene["chain"], template,
            allocation=allocation, guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertFalse(verification["passed"])

    def test_certificate_hash_changes_with_the_path(self):
        scene = _scene()
        base = _certificate_for(scene)
        bent_path = VirtualAPath(
            scene["request"].start_qf, _constant_profile(0.2),
            scene["request"].model)
        bent = _certificate_for(scene, path=bent_path)
        self.assertEqual(base["status"], FORMATION_CERTIFICATE_VERIFIED)
        self.assertEqual(bent["status"], FORMATION_CERTIFICATE_VERIFIED)
        self.assertNotEqual(base["path_sha256"], bent["path_sha256"])
        self.assertNotEqual(
            base["certificate_sha256"], bent["certificate_sha256"])

    def _gate_solve(self, scene, **patches):
        def fake_solve(request, config, packet, guess,
                       wall_limit_s=None, corridor_constraint=None,
                       **kwargs):
            return _fake_ipopt_result(_z(0.0))

        with mock.patch.object(
                optimizer_module, "_solve_with_ipopt",
                side_effect=fake_solve), \
                mock.patch.object(
                    optimizer_module, "audit_geometry") as audit_mock:
            for name, patch in patches.items():
                patcher = mock.patch.object(
                    optimizer_module, name, **patch)
                patcher.start()
                self.addCleanup(patcher.stop)
            result = solve_open_path_guided(
                scene["request"], scene["config"],
                route_guides=(scene["guide"],),
                corridor_chain=scene["chain"],
                corridor_allocation=scene["allocation"],
                formation_template=scene["template"],
                formation_planning_context_sha256=CONTEXT_SHA)
        return result, audit_mock

    def test_optimizer_rejects_unconfirmed_certificate_before_audit(self):
        scene = _scene()
        fake_certificate = {
            "status": FORMATION_CERTIFICATE_UNCONFIRMED,
            "reason": "INTERVAL_NOT_CLOSED", "timed_out": False}
        result, audit_mock = self._gate_solve(
            scene,
            certify_formation_path={"return_value": fake_certificate})
        audit_mock.assert_not_called()
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(result.reason,
                         "FORMATION_CERTIFICATE_UNCONFIRMED")
        self.assertIsNone(result.path)
        attempt = result.attempts[-1]
        self.assertEqual(
            attempt.status,
            "SOLVER_SUCCESS_FORMATION_CERTIFICATE_REJECTED")
        self.assertEqual(attempt.rejection_reason,
                         "FORMATION_CERTIFICATE_UNCONFIRMED")
        self.assertEqual(
            result.formation_certificate["status"],
            FORMATION_CERTIFICATE_UNCONFIRMED)
        self.assertEqual(
            result.as_dict()["formation_certificate"]["status"],
            FORMATION_CERTIFICATE_UNCONFIRMED)

    def test_optimizer_rejects_verified_certificate_failing_verify(self):
        scene = _scene()
        fake_certificate = {
            "status": FORMATION_CERTIFICATE_VERIFIED,
            "reason": "", "timed_out": False}
        failed = {"passed": False,
                  "reason": "CERTIFICATE_HASH_MISMATCH"}
        result, audit_mock = self._gate_solve(
            scene,
            certify_formation_path={"return_value": fake_certificate},
            verify_formation_certificate={"return_value": failed})
        audit_mock.assert_not_called()
        self.assertEqual(result.status, "NO_ACCEPTED_CANDIDATE")
        self.assertEqual(
            result.reason,
            "FORMATION_CERTIFICATE_VERIFICATION_FAILED")
        self.assertEqual(
            result.attempts[-1].status,
            "SOLVER_SUCCESS_FORMATION_CERTIFICATE_REJECTED")
        self.assertEqual(
            result.attempts[-1].rejection_reason,
            "FORMATION_CERTIFICATE_VERIFICATION_FAILED")

    def test_certificate_timeout_stops_with_budget_exhausted(self):
        scene = _scene()
        timed_out = {
            "status": FORMATION_CERTIFICATE_UNCONFIRMED,
            "reason": "TIME_LIMIT", "timed_out": True}
        result, audit_mock = self._gate_solve(
            scene,
            certify_formation_path={"return_value": timed_out})
        audit_mock.assert_not_called()
        self.assertEqual(result.status, "BUDGET_EXHAUSTED")
        self.assertEqual(result.reason, "SOFT_BUDGET_EXHAUSTED")
        self.assertTrue(result.deadline_exhausted)
        self.assertEqual(len(result.attempts), 1)
        self.assertEqual(
            result.attempts[0].rejection_reason,
            "FORMATION_CERTIFICATE_UNCONFIRMED")

    def test_accepted_candidate_carries_verified_certificate(self):
        scene = _scene()
        result, audit_mock = self._gate_solve(scene)
        audit_mock.assert_called_once()
        self.assertEqual(result.status, "GEOMETRY_FEASIBLE",
                         result.as_dict())
        self.assertEqual(result.attempts[-1].status,
                         "SOLVER_SUCCESS_AUDITED")
        certificate = result.formation_certificate
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_VERIFIED)
        payload = result.as_dict()
        self.assertEqual(
            payload["formation_certificate"]["status"],
            FORMATION_CERTIFICATE_VERIFIED)
        verification = verify_formation_certificate(
            payload["formation_certificate"], result.path,
            scene["request"], scene["chain"], scene["template"],
            allocation=scene["allocation"], guide=scene["guide"],
            planning_context_sha256=CONTEXT_SHA)
        self.assertTrue(verification["passed"], verification)

    def test_f_only_result_key_set_keeps_no_certificate(self):
        f_only = GeometryResult(status="NO_MOTION")
        payload = f_only.as_dict()
        self.assertEqual(set(payload), {
            "status", "reason", "detail", "request_sha256",
            "request_summary", "config_summary", "solver_used",
            "goal_heading_branch_rad", "wall_time_s",
            "deadline_exhausted", "deadline_summary", "initial_count",
            "corridor_halfspace_constraint_count",
            "corridor_max_residual_m", "path", "audit", "attempts"})
        union = GeometryResult(
            status="NO_MOTION",
            formation_corridor_scope=(
                FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1),
            formation_certificate={
                "status": FORMATION_CERTIFICATE_VERIFIED})
        union_payload = union.as_dict()
        self.assertEqual(
            union_payload["formation_certificate"]["status"],
            FORMATION_CERTIFICATE_VERIFIED)
        self.assertEqual(
            union_payload["formation_corridor_scope"],
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1)


class FormationHeadingIntervalReworkTests(unittest.TestCase):
    """Rework 01: the whole-interval heading increment must include its start.

    For ``s`` in ``[a, b]`` the increment ``psi(s) - psi(a)`` is the integral
    of the curvature over ``[a, s]``, so it lies in ``kappa([a,b]) * [0, w]``.
    The old ``kappa * w`` bound kept only the interval-end increment and
    excluded the start whenever the curvature had one sign.
    """

    def test_positive_curvature_increment_interval_contains_the_start(self):
        interval = _heading_interval(0.0, (0.4, 0.6), 0.1)
        # Outward rounding: the lower bound is strictly below zero and the
        # upper bound strictly above the end increment.
        self.assertLess(interval[0], 0.0)
        self.assertGreater(interval[1], 0.06)

    def test_negative_curvature_increment_interval_contains_the_start(self):
        interval = _heading_interval(0.0, (-0.6, -0.4), 0.1)
        self.assertLess(interval[0], -0.06)
        self.assertGreater(interval[1], 0.0)

    def test_crossing_zero_increment_interval_contains_both_products(self):
        interval = _heading_interval(0.0, (-0.2, 0.3), 0.1)
        self.assertLess(interval[0], -0.02)
        self.assertGreater(interval[1], 0.03)

    def _profiles(self):
        knots = (0.3, -0.3, 0.3, -0.3, 0.3, -0.3, 0.3, -0.3, 0.3)
        h = float(S_LENGTH) / float(INTERVALS)
        crossing = C2CurvatureProfile.from_knots([
            ControlKnot(index * h, value, 0.0)
            for index, value in enumerate(knots)])
        return (
            ("positive", _constant_profile(0.5)),
            ("negative", _constant_profile(-0.5)),
            ("crossing", crossing))

    def test_dense_partial_integrals_stay_inside_the_heading_interval(self):
        request = _request(0.0, 0.0)
        steps = 97
        for label, profile in self._profiles():
            path = VirtualAPath(
                request.start_qf, profile, request.model)
            angle_error = float(path.angle_error_bound_rad)
            for s0, s1 in path.profile.domain_segments:
                kappa = tuple(
                    float(value)
                    for value in path.profile.interval_bounds(s0, s1)["k"])
                self.assertEqual(len(kappa), 2, label)
                psi_a = float(path.state_at_s(s0, "right").qf.psi)
                heading = inflate_certificate_interval(
                    _heading_interval(psi_a, kappa, s1 - s0),
                    2.0 * angle_error)
                for index in range(1, steps + 1):
                    s = s0 + (s1 - s0) * index / float(steps)
                    psi_s = float(path.state_at_s(s).qf.psi)
                    increment = psi_s - psi_a
                    # ``heading`` encloses the absolute psi(s); equivalently
                    # the increment lies in ``heading - psi(a)``.
                    self.assertGreaterEqual(
                        psi_s, heading[0],
                        "{} s={!r} increment={!r} heading={!r}".format(
                            label, s, increment, heading))
                    self.assertLessEqual(
                        psi_s, heading[1],
                        "{} s={!r} increment={!r} heading={!r}".format(
                            label, s, increment, heading))

    def test_scalar_width_bound_would_exclude_the_dense_increments(self):
        """The pre-rework scalar-width bound misses the interval start.

        It only does so for sign-definite curvature: for a sign-crossing
        curvature interval the two end products already bracket zero, which
        is why the unit-level tests above are the real regression guards.
        """
        request = _request(0.0, 0.0)
        observed = {}
        for label, profile in self._profiles():
            path = VirtualAPath(
                request.start_qf, profile, request.model)
            misses = []
            for s0, s1 in path.profile.domain_segments:
                kappa = tuple(
                    float(value)
                    for value in path.profile.interval_bounds(s0, s1)["k"])
                psi_a = float(path.state_at_s(s0, "right").qf.psi)
                old_lower = psi_a + kappa[0] * (s1 - s0)
                old_upper = psi_a + kappa[1] * (s1 - s0)
                for index in range(1, 17):
                    s = s0 + (s1 - s0) * index / 16.0
                    psi_s = float(path.state_at_s(s).qf.psi)
                    if psi_s < old_lower - 1e-12:
                        misses.append("below")
                    if psi_s > old_upper + 1e-12:
                        misses.append("above")
            observed[label] = misses
        self.assertIn("below", observed["positive"])
        self.assertIn("above", observed["negative"])
        self.assertEqual(observed["crossing"], [])

    def test_side_wall_peak_inside_cut_interval_is_not_verified(self):
        scene = _side_wall_scene()
        # Every finite row station passes, but the continuous OBB maximum
        # crosses the margin-adjusted side wall.
        self.assertLess(scene["station_max_m"], scene["wall_m"])
        self.assertGreater(scene["fine_max_m"], scene["wall_m"])
        certificate = _certificate_for(scene)
        self.assertNotEqual(
            certificate["status"], FORMATION_CERTIFICATE_VERIFIED)
        if certificate["status"] == FORMATION_CERTIFICATE_VIOLATED:
            self.assertEqual(certificate["reason"],
                             "DEFINITE_INTERVAL_VIOLATION")
            witness = certificate["witness"]
            self.assertEqual(witness["kind"], "obb")
            self.assertGreater(witness["residual_m"], 0.0)
            self.assertGreater(witness["evidence_m"], 1e-9)
            self.assertGreater(witness["point_x"],
                               scene["wall_m"] - MARGIN_M)
        else:
            self.assertEqual(certificate["status"],
                             FORMATION_CERTIFICATE_UNCONFIRMED)
            self.assertGreater(certificate["unclosed_interval_count"], 0)
            self.assertIsNone(certificate["witness"])
        verification = verify_formation_certificate(
            certificate, scene["path"], scene["request"], scene["chain"],
            scene["template"], allocation=scene["allocation"],
            guide=scene["guide"], planning_context_sha256=CONTEXT_SHA)
        self.assertFalse(verification["passed"])

    def test_side_wall_with_positive_clearance_is_verified(self):
        scene = _side_wall_scene(clearance_m=0.02)
        certificate = _certificate_for(scene)
        self.assertEqual(certificate["status"],
                         FORMATION_CERTIFICATE_VERIFIED,
                         certificate.get("unclosed_intervals"))
        self.assertEqual(certificate["reason"], "")
        self.assertEqual(certificate["unclosed_interval_count"], 0)
        self.assertLessEqual(certificate["max_outward_residual_m"], 0.0)
        self.assertIsNone(certificate["witness"])
        verification = verify_formation_certificate(
            certificate, scene["path"], scene["request"], scene["chain"],
            scene["template"], allocation=scene["allocation"],
            guide=scene["guide"], planning_context_sha256=CONTEXT_SHA)
        self.assertTrue(verification["passed"], verification)


class FormationFRegionReworkTests(unittest.TestCase):
    """Rework Step B/C: per-template-segment F region assignment.

    The F reference point is no longer compiled from the fixed corridor
    allocation under union scope.  Every template segment carries one audited
    Chain region; the same assignment feeds the template payload, the NLP F
    rows, the continuous certificate and the playback rebuild.
    """

    def _first_f_switch(self, template):
        initial = int(template.segments[0].f_region_index)
        for segment in template.segments:
            if int(segment.f_region_index) != initial:
                return float(segment.sigma_start)
        return 1.0

    def _allocation_index(self, allocation, sigma):
        for left, right, index in allocation:
            if float(left) - 1e-12 <= float(sigma) < float(right) - 1e-12:
                return int(index)
            if float(right) >= 1.0 - 1e-12:
                return int(index)
        return int(allocation[-1][2])

    def _union_constraint(self, scene, template):
        return _build_corridor_constraint(
            scene["chain"], scene["allocation"], scene["request"],
            scene["config"], formation_template=template,
            planning_context_sha256=CONTEXT_SHA,
            formation_guide=scene["guide"])

    def test_f_profiles_adjacent_and_two_sided_on_audited_portal_chain(self):
        scene = _portal_scene(overlap_half=0.3)
        self.assertTrue(scene["chain_audit"]["passed"])
        candidates = _build_portal_candidates(scene)
        self.assertEqual(len(candidates.templates), 3)
        margin = MARGIN_M
        switch_sigmas = []
        for rank, template in enumerate(candidates.templates):
            validate_formation_template(
                template, scene["request"], scene["chain"],
                allocation=scene["allocation"], guide=scene["guide"],
                planning_context_sha256=CONTEXT_SHA)
            segments = template.segments
            self.assertAlmostEqual(segments[0].sigma_start, 0.0, places=12)
            self.assertAlmostEqual(segments[-1].sigma_end, 1.0, places=12)
            for previous, current in zip(segments, segments[1:]):
                self.assertAlmostEqual(
                    previous.sigma_end, current.sigma_start, places=12)
                left = int(previous.f_region_index)
                right = int(current.f_region_index)
                if left == right:
                    # A cover-run split inside one F region is not an F cut.
                    continue
                self.assertEqual(abs(right - left), 1, (rank, previous, current))
                # An F region switch must be two-sided: the selected guide
                # holds in both neighbouring regions at the shared cut.
                pose = _guide_pose_at_sigma(
                    scene["guide"], float(previous.sigma_end))
                point = (float(pose["x"]), float(pose["y"]))
                for region_index in (left, right):
                    self.assertTrue(_region_contains_point(
                        scene["chain"].regions[region_index], point,
                        margin, 1e-9),
                        (rank, region_index, point))
            switch_sigmas.append(self._first_f_switch(template))
        # Fixed deterministic order: allocation, early migration (forward),
        # late stay (backward).
        self.assertLess(switch_sigmas[1], switch_sigmas[0])
        self.assertLess(switch_sigmas[0], switch_sigmas[2])
        again = _build_portal_candidates(scene)
        self.assertEqual(again.as_dict(), candidates.as_dict())

    def test_template_validation_rejects_non_adjacent_and_unknown_f_region(self):
        request = _request()
        regions = (
            _box(-10.0, -3.0, 0.6, 3.0, (0.0, 0.0), (0.0, 0.0)),
            _box(0.0, -3.0, 4.2, 3.0, (0.0, 0.0), (0.0, 0.0)),
            _box(3.6, -3.0, 10.0, 3.0, (0.0, 0.0), (0.0, 0.0)))
        chain = CorridorChain(
            regions=regions, route_source="rework_f_region_fixture",
            grid_sha256=GRID_SHA)
        guide = make_corridor_guide(chain, (0.0, 0.0), (S_LENGTH, 0.0))
        allocation = allocate_corridors(chain, guide, INTERVALS)
        indices = [index for _l, _r, index in allocation]
        self.assertEqual(sorted(set(indices)), [0, 1, 2])
        self.assertEqual(indices, sorted(indices))
        template = single_region_template(
            request, chain, allocation, guide,
            planning_context_sha256=CONTEXT_SHA, region_indices=0,
            f_region_index=0, margin_m=MARGIN_M, margin_source="fixture")
        covers = template.segments[0].covers
        # sigma=0.05 lies in the region-0 allocation entry, so an F assignment
        # to region 2 would skip the directly adjacent region 1.
        non_adjacent = replace(template, segments=(
            FormationCorridorTemplateSegment(
                sigma_start=0.0, sigma_end=0.1, covers=covers,
                f_region_index=2),
            FormationCorridorTemplateSegment(
                sigma_start=0.1, sigma_end=1.0, covers=covers,
                f_region_index=0)))
        with self.assertRaises(VirtualAContractError) as raised:
            validate_formation_template(
                non_adjacent, request, chain, allocation=allocation,
                guide=guide, planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(raised.exception.reason,
                         "INVALID_FORMATION_CORRIDOR_TEMPLATE")
        self.assertIn("not the allocation region or a directly adjacent",
                      str(raised.exception))
        unknown = replace(template, segments=(
            FormationCorridorTemplateSegment(
                sigma_start=0.0, sigma_end=1.0, covers=covers,
                f_region_index=len(chain.regions)),))
        with self.assertRaises(VirtualAContractError) as raised:
            validate_formation_template(
                unknown, request, chain, allocation=allocation,
                guide=guide, planning_context_sha256=CONTEXT_SHA)
        self.assertEqual(raised.exception.reason,
                         "INVALID_FORMATION_CORRIDOR_TEMPLATE")
        self.assertIn("outside the chain", str(raised.exception))

    def test_union_rows_compile_from_template_f_region_not_allocation(self):
        scene = _portal_scene(overlap_half=0.3)
        template = _build_portal_candidates(scene).templates[1]
        constraint = self._union_constraint(scene, template)
        # Union builds must not also append the historical fixed-allocation
        # F-only rows: those live in the legacy corridor groups only.
        self.assertEqual(constraint.sample_count, 0)
        formation = constraint.formation
        self.assertIsNotNone(formation)
        self.assertEqual(
            formation.f_region_by_segment,
            tuple(int(segment.f_region_index)
                  for segment in template.segments))
        self.assertGreater(formation.f_scalar_row_count, 0)
        self.assertGreater(formation.scalar_row_count, 0)
        self.assertEqual(
            formation.f_scalar_row_count,
            sum(len(sample.halfspaces)
                for group in formation.f_groups for sample in group))
        migrated = []
        regions_used = set()
        for group in formation.f_groups:
            for sample in group:
                segment = template.segments[
                    int(sample.template_segment_index)]
                self.assertEqual(int(sample.region_index),
                                 int(segment.f_region_index))
                self.assertTrue(all(
                    int(halfspace.region_index) == int(sample.region_index)
                    for halfspace in sample.halfspaces))
                regions_used.add(int(sample.region_index))
                allocation_index = self._allocation_index(
                    scene["allocation"], float(sample.sigma))
                if int(sample.region_index) != allocation_index:
                    migrated.append(
                        (float(sample.sigma), int(sample.region_index),
                         allocation_index))
        self.assertEqual(regions_used, {0, 1})
        # The forward template really places F in region 1 at sigmas whose
        # fixed allocation index is still 0; this is the row-level migration
        # window the union certificate is allowed to use.
        self.assertTrue(any(region == 1 and allocation == 0
                            for _sigma, region, allocation in migrated),
                        migrated)

    def test_f_rows_shared_by_numeric_casadi_and_scipy_fallback(self):
        if casadi is None:
            self.skipTest("CasADi not installed")
        scene = _portal_scene(overlap_half=0.3)
        template = _build_portal_candidates(scene).templates[1]
        constraint = self._union_constraint(scene, template)
        request, config = scene["request"], scene["config"]
        packet = _search_packet(request, config)
        n = int(packet["intervals"])
        vector = np.asarray(
            [4.0] + [0.0] * (n + 1) + [0.0] * (n + 1), dtype=float)
        equality_end, signed_end, corridor_end = _constraint_value_offsets(
            request, config, packet, constraint)
        numeric = _numeric_constraint_vector(
            vector, request, config, packet,
            int(config.collocation_points), corridor_constraint=constraint)
        nlp, built = _casadi_nlp(request, config, packet, constraint)
        _solver, _lower_x, _upper_x, lower_g, upper_g = built
        g_function = casadi.Function("g", [nlp["x"]], [nlp["g"]])
        symbolic = np.asarray(g_function(vector), dtype=float).ravel()
        self.assertEqual(numeric.size, corridor_end)
        self.assertEqual(symbolic.size, corridor_end)
        np.testing.assert_allclose(symbolic, numeric, atol=1e-8, rtol=0.0)
        formation = constraint.formation
        f_count = int(formation.f_scalar_row_count)
        obb_count = int(formation.scalar_row_count)
        f_slice = slice(corridor_end - obb_count - f_count,
                        corridor_end - obb_count)
        self.assertEqual(
            int(np.count_nonzero(np.isfinite(symbolic[f_slice]))), f_count)
        self.assertTrue(np.all(np.isneginf(lower_g[f_slice])))
        self.assertTrue(np.all(upper_g[f_slice] == 0.0))

        # The SciPy fallback evaluates the very same row vector: patch the
        # one solver entry point and inspect the inequality it would pass.
        guess = InitialGuess(
            source="rework_test", label="rework_f_rows", S=4.0,
            kappas=(0.0,) * (n + 1), etas=(0.0,) * (n + 1),
            length_multiplier=1.0, shape_sign=0,
            metadata={"intervals": n})
        captured = {}

        def fake_minimize(fun, x0, method=None, bounds=None,
                          constraints=None, options=None, callback=None):
            captured["constraints"] = constraints
            return SimpleNamespace(
                x=np.asarray(x0, dtype=float), fun=0.0, message="fake",
                success=False, nit=0)

        with mock.patch("scipy.optimize.minimize",
                        side_effect=fake_minimize):
            result = optimizer_module._solve_with_scipy(
                request, config, packet, guess,
                corridor_constraint=constraint, wall_limit_s=5.0)
        self.assertEqual(result["solver"], optimizer_module.SOLVER_SCIPY)
        inequality = captured["constraints"][1]["fun"]
        signed_count = signed_end - equality_end
        values = np.asarray(inequality(np.asarray(guess.vector)),
                            dtype=float)
        corridor_values = -values[2 * signed_count:]
        np.testing.assert_allclose(
            corridor_values, numeric[signed_end:corridor_end],
            atol=1e-12, rtol=0.0)
        np.testing.assert_allclose(
            corridor_values[-obb_count - f_count:-obb_count],
            numeric[f_slice], atol=0.0, rtol=0.0)

    def test_f_region_is_sealed_in_payload_and_rebuilt_by_verifier(self):
        scene = _portal_scene(overlap_half=0.3)
        template = _build_portal_candidates(scene).templates[1]
        payload = template.as_dict()
        self.assertEqual(payload["schema_version"],
                         FORMATION_CORRIDOR_TEMPLATE_SCHEMA)
        self.assertEqual(
            [int(segment["f_region_index"]) for segment in payload["segments"]],
            [int(segment.f_region_index) for segment in template.segments])
        rebuilt = formation_template_from_payload(
            json.loads(json.dumps(payload)))
        self.assertEqual(rebuilt.template_id, template.template_id)
        self.assertEqual(
            [int(segment.f_region_index) for segment in rebuilt.segments],
            [int(segment.f_region_index) for segment in template.segments])

        tampered = json.loads(json.dumps(payload))
        changed = next(segment for segment in tampered["segments"]
                       if int(segment["f_region_index"]) == 1)
        changed["f_region_index"] = 0
        with self.assertRaises(VirtualAContractError) as raised:
            formation_template_from_payload(tampered)
        self.assertEqual(raised.exception.reason,
                         "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH")

        missing = json.loads(json.dumps(payload))
        del missing["segments"][0]["f_region_index"]
        with self.assertRaises(VirtualAContractError) as raised:
            formation_template_from_payload(missing)
        self.assertEqual(raised.exception.reason,
                         "INVALID_FORMATION_CORRIDOR_TEMPLATE")
        self.assertIn("template segment keys", str(raised.exception))

    def test_f_only_fixed_allocation_rows_remain_and_union_omits_them(self):
        scene = _scene()
        f_only_config = replace_config_scope(
            scene["config"], FORMATION_CORRIDOR_SCOPE_F_ONLY)
        legacy = _build_corridor_constraint(
            scene["chain"], scene["allocation"], scene["request"],
            f_only_config)
        self.assertIsNone(legacy.formation)
        self.assertEqual(
            legacy.sample_count,
            len(scene["allocation"]) * (
                2 + int(f_only_config.corridor_quadrature_points)))
        union = _build_corridor_constraint(
            scene["chain"], scene["allocation"], scene["request"],
            scene["config"], formation_template=scene["template"],
            planning_context_sha256=CONTEXT_SHA,
            formation_guide=scene["guide"])
        self.assertEqual(union.sample_count, 0)
        self.assertIsNotNone(union.formation)
        self.assertGreater(union.formation.f_scalar_row_count, 0)
        self.assertEqual(union.formation.f_region_by_segment, (0,))


def replace_config_scope(config, scope, **changes):
    values = {"formation_corridor_scope": scope}
    values.update(changes)
    return replace(config, **values)


if __name__ == "__main__":
    unittest.main()