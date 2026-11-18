"""V1: exact corridor display export and playback marker construction.

The fixture is the saved, independently audited C1 chain.  No planner,
optimizer or ROS master is started here.
"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    ConvexCorridor,
    CorridorChain,
    PlannerConfig,
    VirtualAContractError,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (  # noqa: E402
    CORRIDOR_DISPLAY_STATE,
    corridor_display_fields,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (  # noqa: E402
    plan_virtual_a,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (  # noqa: E402
    verify_envelope_fields,
)
from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    CLEARED,
    CORRIDOR_EDGE_ALPHA,
    CORRIDOR_EDGE_Z_M,
    CORRIDOR_FILL_ALPHA,
    CORRIDOR_FILL_Z_M,
    GUIDE_ASTAR_NAMESPACE,
    GUIDE_CENTERLINE_NAMESPACE,
    GUIDE_CENTERLINE_WIDTH_M,
    GUIDE_Z_M,
    MODE_COLORS,
    PlaybackValidationError,
    build_corridor_markers,
    build_guide_markers,
    build_markers,
    corridor_display_matches,
    frame_at,
)
from visualization_msgs.msg import Marker  # noqa: E402
import virtual_a_g4_support as g4  # noqa: E402
import virtual_a_g5_support as g5  # noqa: E402

C1_SAMPLE = (ROOT / "evaluation/virtual_a_corridor/C1/"
             "20261117T073007Z/sample_chain.json")
C1_DIGEST = ("d3e838f164a1a1bd32fa6d859da4ab0c5f302a3d0feccbfe13d84a10f6"
             "e6e801")
PLAN_ID = 17
FORMATION_ID = "wedge_3"
GENERATION = 4
CONTEXT_SHA = "c" * 64
_NODE_PATH = (ROOT / "catkin_ws/src/fwis_v3_playback/scripts/"
              "formation_playback.py")
_SPEC = importlib.util.spec_from_file_location(
    "v1_formation_playback_node", _NODE_PATH)
_NODE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_NODE)
_PLANNER_NODE_PATH = (ROOT / "catkin_ws/src/fwis_v3_optimizer/scripts/"
                      "formation_uniform_planner_node.py")
_PLANNER_SPEC = importlib.util.spec_from_file_location(
    "v2_uniform_planner_node", _PLANNER_NODE_PATH)
_PLANNER_NODE = importlib.util.module_from_spec(_PLANNER_SPEC)
_PLANNER_SPEC.loader.exec_module(_PLANNER_NODE)


def _load_c1_chain():
    document = json.loads(C1_SAMPLE.read_text(encoding="utf-8"))
    chain_json = document["chain"]
    regions = tuple(ConvexCorridor(
        seed_start=tuple(region["seed_start"]),
        seed_end=tuple(region["seed_end"]),
        A=tuple(tuple(row) for row in region["A"]),
        b=tuple(region["b"]),
        vertices=tuple(tuple(vertex) for vertex in region["vertices"]))
        for region in chain_json["regions"])
    chain = CorridorChain(
        regions=regions, route_source=chain_json["route_source"],
        grid_sha256=chain_json["grid_sha256"])
    return document, chain


def _identity(chain, **overrides):
    identity = {
        "plan_id": PLAN_ID,
        "formation_id": FORMATION_ID,
        "backend": "virtual_rigid_a",
        "generation": GENERATION,
        "route_id": 0,
        "grid_sha256": chain.grid_sha256,
        "planning_context_sha256": CONTEXT_SHA,
        "model_sha256": None,
    }
    identity.update(overrides)
    return identity


def _display_document(chain, corridors):
    return {
        "state": CORRIDOR_DISPLAY_STATE,
        "plan_id": PLAN_ID,
        "formation_id": FORMATION_ID,
        "backend": "virtual_rigid_a",
        "generation": GENERATION,
        "grid_sha256": chain.grid_sha256,
        "planning_context_sha256": CONTEXT_SHA,
        "model_sha256": None,
        "corridors": list(corridors),
    }


def _job(chain, **overrides):
    job = {
        "state": "PLAYING",
        "backend": "virtual_rigid_a",
        "plan_id": PLAN_ID,
        "formation_id": FORMATION_ID,
        "generation": GENERATION,
        "grid_sha256": chain.grid_sha256,
        "planning_context_sha256": CONTEXT_SHA,
        "model_sha256": None,
    }
    job.update(overrides)
    return job


class _Publisher(object):

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


_V2_CASE = {}


def _arc_grid(radius=7.0, half_width=3.0, resolution=0.05, margin=3.0):
    """Synthetic narrow curved corridor around a 7 m radius."""
    origin = -(radius + half_width + margin)
    size = 2.0 * (radius + half_width + margin)
    count = int(math.ceil(size / resolution))
    occupancy = np.ones((count, count), dtype=bool)
    coordinates = (np.arange(count) + 0.5) * resolution + origin
    grid_x, grid_y = np.meshgrid(coordinates, coordinates)
    radial = np.hypot(grid_x, grid_y)
    free = ((radial >= radius - half_width)
            & (radial <= radius + half_width)
            & (grid_x > -0.2))
    occupancy[free] = False
    return g4.Grid(occupancy, resolution, origin, origin,
                   length=0.738, width=0.5)


def _actual_corridor_case():
    """Solve one real corridor plan; all V2 display tests share its data."""
    if _V2_CASE:
        return _V2_CASE
    grid = _arc_grid()
    angle = -math.pi / 3.0
    start = g4.FormationPose(
        7.0 * math.cos(angle), 7.0 * math.sin(angle),
        angle + math.pi / 2.0)
    angle = math.pi / 3.0
    goal = g4.FormationPose(
        7.0 * math.cos(angle), 7.0 * math.sin(angle),
        angle + math.pi / 2.0)
    request = g4.make_request(
        start, goal,
        start_headings={1: start.psi, 2: start.psi, 3: start.psi})
    map_request = g4.map_request(
        request, grid, planner=PlannerConfig(guidance_mode="corridor"))
    result = plan_virtual_a(map_request)
    if not result.executable:
        raise AssertionError(
            "synthetic corridor plan did not succeed: {} {}".format(
                result.status, result.reason))
    rounds = result.search["corridor"]["rounds"]
    if not rounds:
        raise AssertionError("successful corridor plan has no round")
    route_index = rounds[-1]["route_index"]
    identity = {
        "state": CORRIDOR_DISPLAY_STATE,
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "backend": "virtual_rigid_a",
        "generation": 1,
        "grid_sha256": result.grid_sha256,
        "planning_context_sha256": result.planning_context_sha256,
        "model_sha256": result.model_sha256,
    }
    display = _PLANNER_NODE.UniformPlannerNode._display_from_corridor_round(
        identity, result)
    document = dict(identity)
    document["corridors"] = [display]
    job = {name: identity[name] for name in (
        "plan_id", "formation_id", "backend", "generation",
        "grid_sha256", "planning_context_sha256", "model_sha256")}
    job["state"] = "PLAYING"
    snapshot = g5.snapshot_from_request(request)
    envelope = g5.envelope_for_result(
        result=result, request=request, context=map_request.context(),
        grid_sha256=result.grid_sha256, snapshot=snapshot)
    verified = verify_envelope_fields(envelope)
    _V2_CASE.update({
        "map_request": map_request,
        "result": result,
        "round": rounds[-1],
        "route": result.routes[route_index],
        "identity": identity,
        "display": display,
        "document": document,
        "job": job,
        "envelope": envelope,
        "verified": verified,
        "trajectory": verified.trajectory,
        "plan": verified.plan,
    })
    return _V2_CASE


class CorridorDisplayExportTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.c1, cls.chain = _load_c1_chain()
        cls.display = corridor_display_fields(cls.chain, _identity(cls.chain))

    def test_c1_chain_digest_and_export_identity_are_unchanged(self):
        self.assertEqual(self.chain.digest, C1_DIGEST)
        self.assertEqual(self.c1["chain"]["digest"], C1_DIGEST)
        self.assertEqual(self.display["chain_digest"], C1_DIGEST)
        self.assertEqual(self.display["state"], CORRIDOR_DISPLAY_STATE)
        self.assertEqual(self.display["state"],
                         "CANDIDATE_REFERENCE_POINT_ONLY")
        self.assertNotIn("used_by_optimizer", self.display)
        self.assertIsNone(self.display["model_sha256"])

    def test_every_vertex_and_plane_is_copied_within_1e_9_m(self):
        original = self.c1["chain"]["regions"]
        self.assertEqual(len(self.display["regions"]), len(original))
        for index, (source, exported) in enumerate(
                zip(original, self.display["regions"])):
            self.assertEqual(exported["region_id"], index)
            for name in ("seed_start", "seed_end"):
                for got, want in zip(exported[name], source[name]):
                    self.assertLessEqual(abs(got - want), 1e-9)
            for got, want in zip(exported["b"], source["b"]):
                self.assertLessEqual(abs(got - want), 1e-9)
            self.assertEqual(len(exported["A"]), len(source["A"]))
            for got_row, want_row in zip(exported["A"], source["A"]):
                for got, want in zip(got_row, want_row):
                    self.assertLessEqual(abs(got - want), 1e-9)
            self.assertEqual(len(exported["vertices"]),
                             len(source["vertices"]))
            for got, want in zip(exported["vertices"],
                                 source["vertices"]):
                for got_value, want_value in zip(got, want):
                    self.assertLessEqual(abs(got_value - want_value), 1e-9)

    def test_export_rejects_a_different_grid(self):
        identity = _identity(self.chain, grid_sha256="d" * 64)
        with self.assertRaises(VirtualAContractError):
            corridor_display_fields(self.chain, identity)


class CorridorMarkerTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.c1, cls.chain = _load_c1_chain()
        cls.route0 = corridor_display_fields(cls.chain, _identity(cls.chain))
        cls.document = _display_document(cls.chain, [cls.route0])

    def test_fan_triangulation_reproduces_the_audited_region_area(self):
        markers = build_corridor_markers(self.document)
        self.assertEqual(markers[0].action, "DELETEALL")
        fills = [marker for marker in markers
                 if marker.marker_type == "TRIANGLE_LIST"]
        self.assertEqual(len(fills), len(self.chain.regions))
        for fill, region in zip(fills, self.chain.regions):
            self.assertEqual(fill.namespace,
                             "formation/{}/corridors/route_0/fill".format(
                                 FORMATION_ID))
            self.assertTrue(all(point[2] == CORRIDOR_FILL_Z_M
                                for point in fill.points))
            self.assertEqual(fill.color[3], CORRIDOR_FILL_ALPHA)
            # RViz multiplies TRIANGLE_LIST points by Marker.scale; anything
            # other than 1 would draw a shrunken polygon at the origin.
            self.assertEqual(fill.scale, (1.0, 1.0, 1.0))
            area = 0.0
            points = fill.points
            for index in range(0, len(points), 3):
                (x0, y0, _), (x1, y1, _), (x2, y2, _) = points[index:index + 3]
                area += abs((x1 - x0) * (y2 - y0) -
                            (x2 - x0) * (y1 - y0)) * 0.5
            self.assertLessEqual(
                abs(area - region.area_m2) / region.area_m2, 1e-9)

    def test_edges_are_closed_line_strips_at_the_display_height(self):
        markers = build_corridor_markers(self.document)
        edges = [marker for marker in markers
                 if marker.marker_type == "LINE_STRIP"]
        self.assertEqual(len(edges), len(self.chain.regions))
        for edge, region in zip(edges, self.chain.regions):
            self.assertEqual(edge.namespace,
                             "formation/{}/corridors/route_0/edge".format(
                                 FORMATION_ID))
            self.assertEqual(edge.points[0], edge.points[-1])
            self.assertEqual(len(edge.points), len(region.vertices) + 1)
            self.assertTrue(all(point[2] == CORRIDOR_EDGE_Z_M
                                for point in edge.points))
            self.assertEqual(edge.color[3], CORRIDOR_EDGE_ALPHA)
        for marker in markers:
            self.assertEqual(marker.text, "")

    def test_two_routes_get_deterministic_distinct_colors(self):
        route1 = corridor_display_fields(
            self.chain, _identity(self.chain, route_id=1))
        document = _display_document(self.chain, [self.route0, route1])
        first = build_corridor_markers(document)
        second = build_corridor_markers(copy.deepcopy(document))
        fills0 = [marker for marker in first
                  if marker.marker_type == "TRIANGLE_LIST"
                  and marker.namespace.endswith("route_0/fill")]
        fills1 = [marker for marker in first
                  if marker.marker_type == "TRIANGLE_LIST"
                  and marker.namespace.endswith("route_1/fill")]
        self.assertTrue(fills0 and fills1)
        self.assertNotEqual(fills0[0].color, fills1[0].color)
        self.assertEqual(
            [marker.color for marker in first],
            [marker.color for marker in second])

    def test_replacement_deletes_the_old_layer_before_adding_smaller_one(self):
        old = _display_document(self.chain, [self.route0])
        small_chain = CorridorChain(
            regions=(self.chain.regions[0],),
            route_source=self.chain.route_source,
            grid_sha256=self.chain.grid_sha256)
        small = corridor_display_fields(small_chain, _identity(small_chain))
        new = _display_document(self.chain, [small])
        old_markers = build_corridor_markers(old)
        new_markers = build_corridor_markers(new)
        self.assertEqual(new_markers[0].action, "DELETEALL")
        self.assertTrue(all(marker.action == "ADD"
                            for marker in new_markers[1:]))
        self.assertGreater(len(old_markers), len(new_markers))
        ids = {(marker.namespace, marker.marker_id)
               for marker in new_markers[1:]}
        self.assertEqual(len(ids), len(new_markers) - 1)

    def test_non_convex_reversed_or_claimed_payloads_are_rejected(self):
        reversed_document = copy.deepcopy(self.document)
        reversed_document["corridors"][0]["regions"][0]["vertices"].reverse()
        with self.assertRaises(PlaybackValidationError):
            build_corridor_markers(reversed_document)
        claimed = copy.deepcopy(self.document)
        claimed["used_by_optimizer"] = True
        with self.assertRaises(PlaybackValidationError):
            build_corridor_markers(claimed)
        wrong_state = copy.deepcopy(self.document)
        wrong_state["state"] = "FEASIBLE"
        with self.assertRaises(PlaybackValidationError):
            build_corridor_markers(wrong_state)


class CorridorAdmissionMatchTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.c1, cls.chain = _load_c1_chain()
        cls.route0 = corridor_display_fields(cls.chain, _identity(cls.chain))
        cls.document = _display_document(cls.chain, [cls.route0])
        cls.job = _job(cls.chain)

    def test_matching_admission_is_accepted(self):
        self.assertTrue(corridor_display_matches(self.document, self.job))

    def test_wrong_plan_map_context_or_generation_is_ignored(self):
        for name, value in (
                ("plan_id", PLAN_ID + 1),
                ("formation_id", "other_formation"),
                ("backend", "uniform_v02"),
                ("generation", GENERATION + 1),
                ("grid_sha256", "d" * 64),
                ("planning_context_sha256", "e" * 64),
                ("model_sha256", "f" * 64)):
            with self.subTest(name=name):
                job = dict(self.job)
                job[name] = value
                self.assertFalse(
                    corridor_display_matches(self.document, job))
        document = copy.deepcopy(self.document)
        document["corridors"][0]["grid_sha256"] = "d" * 64
        self.assertFalse(corridor_display_matches(document, self.job))
        self.assertFalse(corridor_display_matches(self.document, None))

    def test_cleared_payload_only_deletes_the_matching_generation(self):
        cleared = {
            "state": CLEARED,
            "plan_id": PLAN_ID,
            "formation_id": FORMATION_ID,
            "backend": "virtual_rigid_a",
            "generation": GENERATION,
            "grid_sha256": None,
            "planning_context_sha256": None,
            "model_sha256": None,
            "corridors": [],
        }
        self.assertTrue(corridor_display_matches(cleared, self.job))
        cleared["generation"] = GENERATION + 1
        self.assertFalse(corridor_display_matches(cleared, self.job))


class CorridorCallbackTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.c1, cls.chain = _load_c1_chain()
        cls.route0 = corridor_display_fields(cls.chain, _identity(cls.chain))
        cls.document = _display_document(cls.chain, [cls.route0])

    def _node(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node._lock = threading.RLock()
        node.frame_id = "map"
        node.backend = "virtual_rigid_a"
        node.transport_ui = False
        node._job_state = "PLAYING"
        node._job_state_plan = PLAN_ID
        node._virtual_jobs = {PLAN_ID: _job(self.chain)}
        node.corridor_pub = _Publisher()
        return node

    def _publish(self, node, document):
        message = SimpleNamespace(data=json.dumps(document))
        with mock.patch.object(
                _NODE.rospy.Time, "now",
                return_value=_NODE.rospy.Time(0, 0)):
            node.on_planning_debug(message)

    def test_matching_payload_publishes_one_atomic_replacement(self):
        node = self._node()
        self._publish(node, self.document)
        self.assertEqual(len(node.corridor_pub.messages), 1)
        markers = node.corridor_pub.messages[0].markers
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertTrue(all(marker.action == Marker.ADD
                            for marker in markers[1:]))

    def test_wrong_generation_never_touches_the_corridor_layer(self):
        node = self._node()
        document = copy.deepcopy(self.document)
        document["generation"] = GENERATION + 1
        self._publish(node, document)
        self.assertEqual(node.corridor_pub.messages, [])

    def test_stale_plan_from_an_old_job_record_is_ignored(self):
        node = self._node()
        old_plan = PLAN_ID - 1
        old_route = corridor_display_fields(
            self.chain, _identity(self.chain, plan_id=old_plan))
        old = _display_document(self.chain, [old_route])
        old["plan_id"] = old_plan
        node._virtual_jobs[old_plan] = _job(self.chain, plan_id=old_plan)
        self._publish(node, old)
        self.assertEqual(node.corridor_pub.messages, [])

    def test_matching_cleared_payload_deletes_without_adding(self):
        node = self._node()
        cleared = {
            "state": CLEARED,
            "plan_id": PLAN_ID,
            "formation_id": FORMATION_ID,
            "backend": "virtual_rigid_a",
            "generation": GENERATION,
            "grid_sha256": None,
            "planning_context_sha256": None,
            "model_sha256": None,
            "corridors": [],
        }
        self._publish(node, cleared)
        self.assertEqual(len(node.corridor_pub.messages), 1)
        markers = node.corridor_pub.messages[0].markers
        self.assertEqual(len(markers), 1)
        self.assertEqual(markers[0].action, Marker.DELETEALL)


class V2ActualCorridorLayersTest(unittest.TestCase):
    """The three layers are copies of one actual corridor planning result."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()

    def _assert_pair(self, got, want, message=""):
        self.assertLessEqual(abs(got - want), 1e-9, message)

    def test_export_copies_actual_route_guide_regions_and_chain_digest(self):
        case = self.case
        display = case["display"]
        round_record = case["round"]
        self.assertEqual(display["route_id"], round_record["route_index"])
        self.assertTrue(display["corridor_used"])
        self.assertEqual(display["chain_sha256"],
                         round_record["chain_digest"])
        self.assertEqual(display["chain_sha256"], display["chain_digest"])
        self.assertEqual(display["guide_quality"],
                         round_record["guide"]["quality"])
        self.assertEqual(display["guide_label"], "PL_GUIDE")
        self.assertEqual(len(display["raw_route_points"]),
                         len(case["route"].points))
        for got, want in zip(display["raw_route_points"],
                             case["route"].points):
            self._assert_pair(got[0], want[0], "A* x")
            self._assert_pair(got[1], want[1], "A* y")
        self.assertEqual(len(display["guide_points"]),
                         len(round_record["guide"]["points"]))
        for got, want in zip(display["guide_points"],
                             round_record["guide"]["points"]):
            self._assert_pair(got[0], want[0], "guide x")
            self._assert_pair(got[1], want[1], "guide y")
        self.assertEqual(len(display["regions"]),
                         len(round_record["chain"]["regions"]))
        for got, want in zip(display["regions"],
                             round_record["chain"]["regions"]):
            for name in ("seed_start", "seed_end"):
                self._assert_pair(got[name][0], want[name][0])
                self._assert_pair(got[name][1], want[name][1])
            self.assertEqual(len(got["vertices"]), len(want["vertices"]))
            for got_point, want_point in zip(got["vertices"],
                                             want["vertices"]):
                self._assert_pair(got_point[0], want_point[0])
                self._assert_pair(got_point[1], want_point[1])
            for got_value, want_value in zip(got["b"], want["b"]):
                self._assert_pair(got_value, want_value)

    def test_guide_markers_copy_route_and_centreline_without_resampling(self):
        case = self.case
        markers = build_guide_markers(case["document"])
        self.assertEqual(markers[0].action, "DELETEALL")
        astar = next(marker for marker in markers
                     if marker.namespace == GUIDE_ASTAR_NAMESPACE
                     and marker.marker_type == "LINE_STRIP")
        centerline = next(marker for marker in markers
                          if marker.namespace == GUIDE_CENTERLINE_NAMESPACE
                          and marker.marker_type == "LINE_STRIP")
        self.assertEqual(len(astar.points), len(case["route"].points))
        for got, want in zip(astar.points, case["route"].points):
            self.assertEqual(got[2], GUIDE_Z_M)
            self._assert_pair(got[0], want[0], "A* display x")
            self._assert_pair(got[1], want[1], "A* display y")
        self.assertEqual(len(centerline.points),
                         len(case["round"]["guide"]["points"]))
        for got, want in zip(centerline.points,
                             case["round"]["guide"]["points"]):
            self.assertEqual(got[2], GUIDE_Z_M)
            self._assert_pair(got[0], want[0], "centreline display x")
            self._assert_pair(got[1], want[1], "centreline display y")
        texts = [marker.text for marker in markers
                 if marker.marker_type == "TEXT_VIEW_FACING"]
        self.assertIn("A* ROUTE | REFERENCE_ONLY", texts)
        self.assertIn("PL_GUIDE | REFERENCE_ONLY", texts)
        self.assertFalse(any(word in text for text in texts
                             for word in ("SMOOTH_PATH", "FEASIBLE",
                                          "CERTIFIED", "AUDIT_PASS")))

    def test_corridor_markers_copy_actual_region_vertices_within_1e_9(self):
        case = self.case
        markers = build_corridor_markers(case["document"])
        fills = [marker for marker in markers
                 if marker.marker_type == "TRIANGLE_LIST"]
        edges = [marker for marker in markers
                 if marker.marker_type == "LINE_STRIP"]
        self.assertEqual(len(fills), len(case["round"]["chain"]["regions"]))
        self.assertEqual(len(edges), len(fills))
        for fill, edge, region in zip(
                fills, edges, case["round"]["chain"]["regions"]):
            vertices = region["vertices"]
            expected = []
            for index in range(1, len(vertices) - 1):
                for point in (vertices[0], vertices[index],
                              vertices[index + 1]):
                    expected.append((point[0], point[1], CORRIDOR_FILL_Z_M))
            self.assertEqual(len(fill.points), len(expected))
            for got, want in zip(fill.points, expected):
                self._assert_pair(got[0], want[0], "corridor fill x")
                self._assert_pair(got[1], want[1], "corridor fill y")
                self.assertEqual(got[2], want[2])
            self.assertEqual(edge.color[3], CORRIDOR_EDGE_ALPHA)
            self.assertEqual(edge.points[-1][0], vertices[0][0])
            self.assertEqual(edge.points[-1][1], vertices[0][1])

    def test_final_paths_keep_mode_colors_and_non_sample_frame_consistency(
            self):
        case = self.case
        plan = case["plan"]
        trajectory = case["trajectory"]
        # 0.5003 is deliberately not a published sample instant.
        elapsed = 0.5003 * plan.duration_s + 1e-5
        frame = frame_at(trajectory, elapsed)
        state = plan.state_at(elapsed)
        self.assertEqual(frame.state, "PLAYING")
        self.assertEqual(frame.frame_id, "map")
        self.assertLessEqual(
            abs(frame.formation_x - state.qf.x), 1e-7)
        self.assertLessEqual(
            abs(frame.formation_y - state.qf.y), 1e-7)
        for vehicle in frame.vehicles:
            expected = state.vehicle(vehicle.vehicle_id)
            self.assertLessEqual(abs(vehicle.center_x - expected.x), 1e-7)
            self.assertLessEqual(abs(vehicle.center_y - expected.y), 1e-7)
            self.assertLessEqual(
                abs(math.atan2(math.sin(vehicle.theta - expected.theta),
                               math.cos(vehicle.theta - expected.theta))),
                1e-7)
        markers, namespaces, _, _ = build_markers(trajectory, frame)
        paths = [marker for marker in markers
                 if marker.colors and marker.marker_type == "LINE_STRIP"]
        self.assertEqual(len(paths), 3)
        for vehicle_id, marker in zip((1, 2, 3), paths):
            samples = trajectory.points[vehicle_id]
            self.assertEqual(len(marker.points), len(samples))
            for got, sample in zip(marker.points, samples):
                self._assert_pair(got[0], sample.center_x)
                self._assert_pair(got[1], sample.center_y)
            self.assertEqual(
                marker.colors,
                tuple(MODE_COLORS[sample.mode] for sample in samples))
            self.assertGreater(marker.scale[0],
                               GUIDE_CENTERLINE_WIDTH_M)
        self.assertFalse(any(namespace.startswith("formation/guides")
                             for namespace in namespaces))

    def test_candidate_and_final_identity_do_not_mix(self):
        case = self.case
        document = case["document"]
        job = case["job"]
        self.assertTrue(corridor_display_matches(document, job))
        self.assertEqual(case["trajectory"].plan_id,
                         case["display"]["plan_id"])
        self.assertEqual(case["trajectory"].formation_id,
                         case["display"]["formation_id"])
        self.assertEqual(case["trajectory"].model_sha256,
                         case["display"]["model_sha256"])
        wrong_model = copy.deepcopy(document)
        wrong_model["model_sha256"] = "f" * 64
        wrong_model["planning_context_sha256"] = "f" * 64
        self.assertFalse(corridor_display_matches(wrong_model, job))
        mixed_entry = copy.deepcopy(document)
        mixed_entry["corridors"][0]["model_sha256"] = "f" * 64
        self.assertFalse(corridor_display_matches(mixed_entry, job))
        with self.assertRaises(PlaybackValidationError):
            build_guide_markers(mixed_entry)
        candidate = copy.deepcopy(document)
        candidate["corridors"][0]["corridor_used"] = False
        markers = build_guide_markers(candidate)
        namespaces = {marker.namespace for marker in markers}
        self.assertIn("formation/guides/astar_candidate", namespaces)
        self.assertIn("formation/guides/centerline_candidate", namespaces)
        self.assertTrue(all("CANDIDATE" in marker.text
                            for marker in markers
                            if marker.marker_type == "TEXT_VIEW_FACING"))

    def test_unvalidated_guide_and_fake_audit_never_get_safety_labels(self):
        case = self.case
        variants = []
        bad_warning = copy.deepcopy(case["document"])
        bad_warning["corridors"][0]["guide_quality"][
            "warnings"] = ["NEGATIVE_DOT_PRODUCT_PRESERVED"]
        variants.append(bad_warning)
        bad_digest = copy.deepcopy(case["document"])
        bad_digest["corridors"][0]["guide_quality"][
            "chain_digest"] = "f" * 64
        variants.append(bad_digest)
        claimed_executable = copy.deepcopy(case["document"])
        claimed_executable["corridors"][0]["guide_quality"][
            "is_executable_path"] = True
        variants.append(claimed_executable)
        safety = ("SMOOTH_PATH", "FEASIBLE", "CERTIFIED", "AUDIT_PASS",
                  "THREE_VEHICLE_CERTIFIED")
        for document in variants:
            markers = build_guide_markers(document)
            texts = [marker.text for marker in markers
                     if marker.marker_type == "TEXT_VIEW_FACING"]
            self.assertTrue(any("UNVALIDATED_GUIDE" in text
                                for text in texts), texts)
            self.assertFalse(any(word in text for text in texts
                                 for word in safety), texts)
        fake = copy.deepcopy(case["document"])
        fake["status"] = "SUCCESS"
        fake["feasible"] = True
        fake["executable"] = True
        fake["trajectory_published"] = True
        fake["certification"] = "FEASIBLE"
        fake["audit_status"] = "PASS"
        fake["corridors"][0].update({
            "feasible": True,
            "executable": True,
            "audit_status": "PASS",
            "status": "SUCCESS",
        })
        texts = [marker.text for marker in build_guide_markers(fake)
                 if marker.marker_type == "TEXT_VIEW_FACING"]
        self.assertFalse(any(word in text for text in texts
                             for word in safety), texts)
        fake_status = {
            "status": "SUCCESS",
            "feasible": True,
            "executable": True,
            "certification": "FEASIBLE",
            "audit_status": "PASS",
        }
        frame = frame_at(case["trajectory"], 0.5 * case["plan"].duration_s)
        _, _, audit_state, executable = build_markers(
            case["trajectory"], frame, status=fake_status)
        self.assertFalse(executable)
        self.assertEqual(audit_state, "GEOMETRY_ONLY")

    def test_layer_toggles_do_not_change_planning_playback_or_frames(self):
        case = self.case
        document_before = copy.deepcopy(case["document"])
        model_before = case["result"].model_sha256
        guide_before = copy.deepcopy(
            case["round"]["guide"])
        elapsed = 0.5003 * case["plan"].duration_s + 1e-5
        frame_before = frame_at(case["trajectory"], elapsed)
        markers_before = build_markers(case["trajectory"], frame_before)
        guide_markers = build_guide_markers(case["document"])
        build_corridor_markers(case["document"])
        # A layer toggle only filters the already-built specs; drawing does
        # not enter planning or the common playback clock.
        only_centreline = tuple(
            marker for marker in guide_markers
            if marker.namespace == GUIDE_CENTERLINE_NAMESPACE)
        self.assertTrue(only_centreline)
        frame_after = frame_at(case["trajectory"], elapsed)
        markers_after = build_markers(case["trajectory"], frame_after)
        self.assertEqual(frame_before, frame_after)
        self.assertEqual(markers_before, markers_after)
        self.assertEqual(case["document"], document_before)
        self.assertEqual(case["round"]["guide"], guide_before)
        self.assertEqual(case["result"].model_sha256, model_before)
        self.assertEqual(frame_after.frame_id, "map")
        self.assertFalse(any(marker.namespace.lower().endswith("/tf")
                             for marker in guide_markers))
        self.assertEqual(
            case["trajectory"].duration,
            verify_envelope_fields(case["envelope"]).trajectory.duration)


class V2PlannerDebugPublicationTest(unittest.TestCase):
    """The planner emits V2 route/guide fields in the same debug payload."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()

    def _node(self):
        node = object.__new__(_PLANNER_NODE.UniformPlannerNode)
        node.backend = "virtual_rigid_a"
        node.planning_debug_pub = _Publisher()
        return node

    def _document(self, node):
        self.assertEqual(len(node.planning_debug_pub.messages), 1)
        return json.loads(node.planning_debug_pub.messages[0].data)

    def test_planning_debug_carries_actual_route_guide_and_chain(self):
        node = self._node()
        node._publish_corridor_debug(
            1, self.case["identity"]["plan_id"],
            SimpleNamespace(formation_id=self.case["identity"]["formation_id"]),
            self.case["result"])
        document = self._document(node)
        self.assertEqual(document["state"], CORRIDOR_DISPLAY_STATE)
        self.assertEqual(len(document["corridors"]), 1)
        corridor = document["corridors"][0]
        for name in ("raw_route_points", "guide_points", "guide_quality",
                     "route_id", "corridor_used", "chain_sha256"):
            self.assertIn(name, corridor)
        self.assertEqual(corridor["route_id"],
                         self.case["round"]["route_index"])
        self.assertTrue(corridor["corridor_used"])
        self.assertEqual(corridor["chain_sha256"],
                         self.case["round"]["chain_digest"])
        self.assertEqual(len(corridor["raw_route_points"]),
                         len(self.case["route"].points))
        self.assertEqual(len(corridor["guide_points"]),
                         len(self.case["round"]["guide"]["points"]))

    def test_missing_corridor_round_clears_instead_of_reusing_old_geometry(
            self):
        node = self._node()
        stale = SimpleNamespace(
            search={},
            routes=(),
            grid_sha256=self.case["identity"]["grid_sha256"],
            planning_context_sha256=self.case["identity"][
                "planning_context_sha256"],
            model_sha256=self.case["identity"]["model_sha256"])
        node._publish_corridor_debug(
            1, self.case["identity"]["plan_id"],
            SimpleNamespace(formation_id=self.case["identity"]["formation_id"]),
            stale)
        document = self._document(node)
        self.assertEqual(document["state"], CLEARED)
        self.assertEqual(document["corridors"], [])


class V2GuideCallbackTest(unittest.TestCase):
    """Planning debug replaces or clears both display layers atomically."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()

    def _node(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node._lock = threading.RLock()
        node.frame_id = "map"
        node.backend = "virtual_rigid_a"
        node.transport_ui = False
        node._job_state = "PLAYING"
        node._job_state_plan = self.case["identity"]["plan_id"]
        node._virtual_jobs = {
            self.case["identity"]["plan_id"]: self.case["job"]}
        node.corridor_pub = _Publisher()
        node.guide_pub = _Publisher()
        return node

    def _publish(self, node, document):
        with mock.patch.object(
                _NODE.rospy.Time, "now",
                return_value=_NODE.rospy.Time(0, 0)):
            node.on_planning_debug(SimpleNamespace(data=json.dumps(document)))

    def test_valid_payload_replaces_both_layers_once(self):
        node = self._node()
        self._publish(node, self.case["document"])
        self.assertEqual(len(node.corridor_pub.messages), 1)
        self.assertEqual(len(node.guide_pub.messages), 1)
        corridor_markers = node.corridor_pub.messages[0].markers
        guide_markers = node.guide_pub.messages[0].markers
        self.assertEqual(corridor_markers[0].action, Marker.DELETEALL)
        self.assertEqual(guide_markers[0].action, Marker.DELETEALL)
        self.assertTrue(any(marker.ns == GUIDE_ASTAR_NAMESPACE
                            for marker in guide_markers))
        self.assertTrue(any(marker.ns == GUIDE_CENTERLINE_NAMESPACE
                            for marker in guide_markers))

    def test_cleared_payload_deletes_both_layers(self):
        node = self._node()
        cleared = {
            "state": CLEARED,
            "plan_id": self.case["identity"]["plan_id"],
            "formation_id": self.case["identity"]["formation_id"],
            "backend": "virtual_rigid_a",
            "generation": self.case["identity"]["generation"],
            "grid_sha256": None,
            "planning_context_sha256": None,
            "model_sha256": None,
            "corridors": [],
        }
        self._publish(node, cleared)
        for publisher in (node.corridor_pub, node.guide_pub):
            self.assertEqual(len(publisher.messages), 1)
            self.assertEqual(publisher.messages[0].markers[0].action,
                             Marker.DELETEALL)

    def test_invalid_guide_clears_both_layers_without_stale_add(self):
        node = self._node()
        broken = copy.deepcopy(self.case["document"])
        broken["corridors"][0]["raw_route_points"] = [[0.0, 0.0]]
        self._publish(node, broken)
        for publisher in (node.corridor_pub, node.guide_pub):
            self.assertEqual(len(publisher.messages), 1)
            markers = publisher.messages[0].markers
            self.assertEqual([marker.action for marker in markers],
                             [Marker.DELETEALL])

    def test_wrong_generation_never_touches_either_layer(self):
        node = self._node()
        wrong = copy.deepcopy(self.case["document"])
        wrong["generation"] += 1
        self._publish(node, wrong)
        self.assertEqual(node.corridor_pub.messages, [])
        self.assertEqual(node.guide_pub.messages, [])

    def test_missing_job_digest_is_filled_only_from_verified_envelope(self):
        # The PLANNING job report may predate the planner's success record and
        # still carry model_sha256=None.  The receiver has already verified
        # this plan's envelope, so that one missing field may be supplied.
        node = self._node()
        plan_id = self.case["identity"]["plan_id"]
        job = dict(node._virtual_jobs[plan_id])
        job["model_sha256"] = None
        node._virtual_jobs = {plan_id: job}
        node.trajectory = SimpleNamespace(plan_id=plan_id)
        node._playback_model_sha256 = self.case["identity"]["model_sha256"]
        self._publish(node, self.case["document"])
        self.assertEqual(len(node.corridor_pub.messages), 1)
        self.assertEqual(len(node.guide_pub.messages), 1)
        self.assertTrue(any(
            marker.ns == GUIDE_CENTERLINE_NAMESPACE
            for marker in node.guide_pub.messages[0].markers))

    def test_other_plans_verified_digest_never_fills_job_identity(self):
        node = self._node()
        plan_id = self.case["identity"]["plan_id"]
        job = dict(node._virtual_jobs[plan_id])
        job["model_sha256"] = None
        node._virtual_jobs = {plan_id: job}
        node.trajectory = SimpleNamespace(plan_id=plan_id + 1)
        node._playback_model_sha256 = self.case["identity"]["model_sha256"]
        self._publish(node, self.case["document"])
        self.assertEqual(node.corridor_pub.messages, [])
        self.assertEqual(node.guide_pub.messages, [])


if __name__ == "__main__":
    unittest.main()