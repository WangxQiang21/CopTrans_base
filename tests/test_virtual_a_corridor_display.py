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
    PROGRESS_FAILED_STAGE,
    PROGRESS_SCHEMA,
    PROGRESS_STAGE_INDEX,
    PROGRESS_STATES,
    VirtualAContractError,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (  # noqa: E402
    CORRIDOR_DISPLAY_STATE,
    corridor_display_fields,
)
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    pipeline as pipeline_module,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (  # noqa: E402
    plan_virtual_a,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (  # noqa: E402
    verify_envelope_fields,
)
from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    CANDIDATE_CENTER_NAMESPACE,
    CANDIDATE_CERTIFIED,
    CANDIDATE_DIAGNOSTIC_LABEL,
    CANDIDATE_F_NAMESPACE,
    CANDIDATE_OBB_COLOR,
    CANDIDATE_OBB_LIMIT,
    CANDIDATE_OBB_NAMESPACE,
    CANDIDATE_POINT_LIMIT,
    CANDIDATE_SAMPLE_SPACING_M,
    CANDIDATE_SCHEMA,
    CANDIDATE_UNVERIFIED,
    CLEARED,
    CORRIDOR_EDGE_ALPHA,
    CORRIDOR_EDGE_COLOR,
    CORRIDOR_EDGE_NAMESPACE,
    CORRIDOR_EDGE_Z_M,
    CORRIDOR_FILL_ALPHA,
    CORRIDOR_FILL_COLOR,
    CORRIDOR_FILL_NAMESPACE,
    CORRIDOR_FILL_Z_M,
    PLANNING_ASTAR_NAMESPACE,
    PLANNING_DEBUG_SCHEMA,
    PLANNING_FAILURE_NAMESPACE,
    PLANNING_GUIDE_NAMESPACE,
    PLANNING_PUBLISHED_STATE,
    PLANNING_STATE,
    PlaybackValidationError,
    build_candidate_display,
    build_corridor_markers,
    build_guide_display,
    build_markers,
    corridor_display_matches,
    frame_at,
    sample_candidate_path,
    vehicle_path_samples,
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
    progress_events = []
    result = plan_virtual_a(
        map_request, progress_callback=progress_events.append)
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
        "progress_events": progress_events,
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
        self.assertEqual(CORRIDOR_FILL_ALPHA, 0.09)
        self.assertEqual(CORRIDOR_FILL_COLOR,
                         (232 / 255, 121 / 255, 134 / 255))
        markers = build_corridor_markers(self.document)
        self.assertEqual(markers[0].action, "DELETEALL")
        fills = [marker for marker in markers
                 if marker.marker_type == "TRIANGLE_LIST"]
        self.assertEqual(len(fills), len(self.chain.regions))
        for fill, region in zip(fills, self.chain.regions):
            self.assertEqual(fill.namespace, CORRIDOR_FILL_NAMESPACE)
            self.assertEqual(fill.color,
                             CORRIDOR_FILL_COLOR + (CORRIDOR_FILL_ALPHA,))
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
        self.assertEqual(CORRIDOR_EDGE_ALPHA, 0.55)
        self.assertEqual(CORRIDOR_EDGE_COLOR,
                         (180 / 255, 83 / 255, 98 / 255))
        markers = build_corridor_markers(self.document)
        edges = [marker for marker in markers
                 if marker.marker_type == "LINE_STRIP"]
        self.assertEqual(len(edges), len(self.chain.regions))
        for edge, region in zip(edges, self.chain.regions):
            self.assertEqual(edge.namespace, CORRIDOR_EDGE_NAMESPACE)
            self.assertEqual(edge.color,
                             CORRIDOR_EDGE_COLOR + (CORRIDOR_EDGE_ALPHA,))
            self.assertEqual(edge.points[0], edge.points[-1])
            self.assertEqual(len(edge.points), len(region.vertices) + 1)
            self.assertTrue(all(point[2] == CORRIDOR_EDGE_Z_M
                                for point in edge.points))
            self.assertEqual(edge.color[3], CORRIDOR_EDGE_ALPHA)
            self.assertEqual(edge.scale, (0.013, 0.0, 0.0))
        for marker in markers:
            self.assertEqual(marker.text, "")

    def test_routes_share_the_fixed_color_and_stable_namespace(self):
        route1 = corridor_display_fields(
            self.chain, _identity(self.chain, route_id=1))
        document = _display_document(self.chain, [self.route0, route1])
        first = build_corridor_markers(document)
        second = build_corridor_markers(copy.deepcopy(document))
        fills = [marker for marker in first
                 if marker.marker_type == "TRIANGLE_LIST"]
        self.assertTrue(fills)
        # TASK 01 fixes one red corridor colour; routes must not get a
        # per-route palette and the namespace must not encode formation or
        # route identity.
        self.assertTrue(all(marker.namespace == CORRIDOR_FILL_NAMESPACE
                            for marker in fills))
        self.assertTrue(all(marker.color == CORRIDOR_FILL_COLOR +
                            (CORRIDOR_FILL_ALPHA,) for marker in fills))
        ids = {(marker.namespace, marker.marker_id) for marker in fills}
        self.assertEqual(len(ids), len(fills))
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

    def test_guide_paths_copy_route_and_centreline_without_resampling(self):
        case = self.case
        markers, paths = build_guide_display(case["document"])
        route_id = case["display"]["route_id"]
        self.assertEqual(markers[0].action, "DELETEALL")
        source = [(float(point[0]), float(point[1]))
                  for point in case["route"].points]
        guide_source = [(float(point[0]), float(point[1]))
                        for point in case["round"]["guide"]["points"]]
        astar = paths[("astar_raw", route_id)]
        centreline = paths[("portal_guide", route_id)]
        self.assertEqual(len(astar), len(source))
        for got, want in zip(astar, source):
            self._assert_pair(got[0], want[0], "A* path x")
            self._assert_pair(got[1], want[1], "A* path y")
        self.assertEqual(len(centreline), len(guide_source))
        for got, want in zip(centreline, guide_source):
            self._assert_pair(got[0], want[0], "centreline path x")
            self._assert_pair(got[1], want[1], "centreline path y")
        # Path displays own colour/width; no route line remains in markers.
        self.assertFalse(any(
            marker.marker_type in ("LINE_LIST", "LINE_STRIP")
            for marker in markers))
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

    def test_final_path_geometry_and_non_sample_frame_consistency(self):
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
        # The full centre path is a separate nav_msgs/Path per vehicle (RViz
        # owns its colour/width); the marker array keeps only the current
        # pose, OBB, heading, history and labels.
        paths = vehicle_path_samples(trajectory)
        self.assertEqual(set(paths), {1, 2, 3})
        for vehicle_id in (1, 2, 3):
            samples = trajectory.points[vehicle_id]
            self.assertEqual(len(paths[vehicle_id]), len(samples))
            for got, sample in zip(paths[vehicle_id], samples):
                self._assert_pair(got[0], sample.center_x)
                self._assert_pair(got[1], sample.center_y)
            namespace = "formation/{}/vehicle_{}".format(
                trajectory.formation_id, vehicle_id)
            vehicle_markers = [
                marker for marker in markers
                if marker.namespace == namespace]
            self.assertEqual(len(vehicle_markers), 4)
            self.assertFalse(any(marker.colors
                                 for marker in vehicle_markers))
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
            build_guide_display(mixed_entry)
        candidate = copy.deepcopy(document)
        candidate["corridors"][0]["corridor_used"] = False
        markers, paths = build_guide_display(candidate)
        route_id = case["display"]["route_id"]
        self.assertEqual(
            set(paths),
            {("astar_raw", route_id), ("portal_guide", route_id)})
        namespaces = {marker.namespace for marker in markers}
        self.assertIn(PLANNING_ASTAR_NAMESPACE, namespaces)
        self.assertIn(PLANNING_GUIDE_NAMESPACE, namespaces)
        self.assertFalse(any(namespace.startswith("formation/guides")
                             for namespace in namespaces))
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
            markers = build_guide_display(document)[0]
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
        texts = [marker.text for marker in build_guide_display(fake)[0]
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
        guide_markers, guide_paths = build_guide_display(case["document"])
        build_corridor_markers(case["document"])
        # A layer toggle only filters the already-built specs; drawing does
        # not enter planning or the common playback clock.
        only_centreline = guide_paths[
            ("portal_guide", case["display"]["route_id"])]
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
    """The planner emits progressive documents from the actual result."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()

    def _node(self, astar=None):
        node = object.__new__(_PLANNER_NODE.UniformPlannerNode)
        node.backend = "virtual_rigid_a"
        node.planning_debug_pub = _Publisher()
        result = self.case["result"]
        node._virtual_admission = {
            "formation_id": self.case["identity"]["formation_id"],
            "request_sha256": result.request_sha256,
            "grid_sha256": result.grid_sha256,
            "planning_context_sha256": result.planning_context_sha256,
        }
        node._virtual_request = self.case["map_request"].request
        node._progress_layers = {
            1: {"astar": astar, "corridors": None, "candidates": None}}
        node._planning_progress = None
        node._progress_display_errors = 0
        node._progress_display_disabled = set()
        return node

    def _snapshot(self):
        return SimpleNamespace(
            formation_id=self.case["identity"]["formation_id"])

    def _document(self, node):
        self.assertEqual(len(node.planning_debug_pub.messages), 1)
        return json.loads(node.planning_debug_pub.messages[0].data)

    def test_published_document_carries_actual_route_guide_and_chain(self):
        route = self.case["route"]
        astar = [{"route_id": 0,
                  "points": [[float(p[0]), float(p[1])]
                             for p in route.points]}]
        node = self._node(astar=astar)
        result = self.case["result"]
        node._publish_planning_published(
            1, self.case["identity"]["plan_id"], self._snapshot(), result,
            result.model_sha256, 0.5)
        document = self._document(node)
        self.assertEqual(document["schema"], PLANNING_DEBUG_SCHEMA)
        self.assertEqual(document["state"], PLANNING_PUBLISHED_STATE)
        self.assertEqual(document["stage"], PLANNING_PUBLISHED_STATE)
        self.assertEqual(document["stage_index"], 8)
        self.assertEqual(document["model_sha256"], result.model_sha256)
        self.assertEqual(document["astar"], astar)
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
        candidates = document["candidates"]
        self.assertEqual(candidates["schema"], CANDIDATE_SCHEMA)
        self.assertEqual(candidates["classification"], CANDIDATE_CERTIFIED)
        self.assertTrue(candidates["certified"])

    def test_missing_corridor_round_clears_only_that_layer(self):
        node = self._node()
        stale = SimpleNamespace(
            search={},
            routes=(),
            grid_sha256=self.case["identity"]["grid_sha256"],
            planning_context_sha256=self.case["identity"][
                "planning_context_sha256"],
            model_sha256="f" * 64)
        with mock.patch.object(_PLANNER_NODE.rospy, "logerr"):
            node._publish_planning_published(
                1, self.case["identity"]["plan_id"], self._snapshot(), stale,
                "f" * 64, 0.5)
        document = self._document(node)
        # A display construction failure never changes the plan outcome; it
        # clears only the corridor/guide layer of a PUBLISHED document.
        self.assertEqual(document["state"], PLANNING_PUBLISHED_STATE)
        self.assertIsNone(document["corridors"])
        self.assertEqual(node._progress_display_errors, 1)


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
        node.guide_path_pubs = {
            "/formation/path/astar_raw_0": _Publisher(),
            "/formation/path/portal_guide_0": _Publisher(),
            "/formation/path/astar_active": _Publisher(),
            "/formation/path/portal_guide_active": _Publisher(),
        }
        node.candidate_path_pubs = {}
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
        self.assertTrue(any(marker.ns == PLANNING_ASTAR_NAMESPACE
                            for marker in guide_markers))
        self.assertTrue(any(marker.ns == PLANNING_GUIDE_NAMESPACE
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

    def test_invalid_guide_clears_only_the_guide_layer(self):
        node = self._node()
        broken = copy.deepcopy(self.case["document"])
        broken["corridors"][0]["raw_route_points"] = [[0.0, 0.0]]
        self._publish(node, broken)
        # The corridor payload is still valid; only the guide builder fails,
        # so only the guide layer is cleared.
        self.assertEqual(len(node.corridor_pub.messages), 1)
        corridor_markers = node.corridor_pub.messages[0].markers
        self.assertEqual(corridor_markers[0].action, Marker.DELETEALL)
        self.assertTrue(any(marker.action == Marker.ADD
                            for marker in corridor_markers[1:]))
        self.assertEqual(len(node.guide_pub.messages), 1)
        markers = node.guide_pub.messages[0].markers
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
            marker.ns == PLANNING_GUIDE_NAMESPACE
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


class ProgressCallbackContractTest(unittest.TestCase):
    """TASK 01 Step 1: the ROS-free progress events of one real solve."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()

    def test_real_success_events_are_monotone_read_only_and_identifiable(self):
        events = self.case["progress_events"]
        map_request = self.case["map_request"]
        self.assertTrue(events)
        seen = []
        first_seen = {}
        last_stage = -1
        for event in events:
            self.assertEqual(event["schema"], PROGRESS_SCHEMA)
            self.assertIn(event["state"], PROGRESS_STATES)
            self.assertEqual(event["request_sha256"],
                             map_request.request_sha256)
            self.assertEqual(event["grid_sha256"], map_request.grid_sha256)
            self.assertGreaterEqual(event["elapsed_s"], 0.0)
            self.assertNotEqual(event["state"], "PUBLISHED")
            if event["stage"] == PROGRESS_FAILED_STAGE:
                self.assertEqual(event["state"], "FAILED")
                self.assertGreaterEqual(event["stage_index"], 0)
                self.assertLessEqual(event["stage_index"], 8)
                continue
            self.assertEqual(event["stage_index"],
                             PROGRESS_STAGE_INDEX[event["stage"]])
            # Real re-entry is allowed across routes/rounds; the stage index
            # may never jump beyond the final audit stage.
            self.assertGreaterEqual(event["stage_index"], 0)
            self.assertLessEqual(event["stage_index"], 8)
            last_stage = event["stage_index"]
            if event["stage"] not in seen:
                seen.append(event["stage"])
                first_seen[event["stage"]] = event["stage_index"]
        self.assertGreaterEqual(last_stage, 0)
        expected = ("ASTAR_SEARCH", "CORRIDOR_BUILD", "PORTAL_GUIDE",
                    "NLP_SOLVE", "GEOMETRY_AUDIT", "DYNAMICS_AUDIT",
                    "OBB_COLLISION_AUDIT")
        for stage in expected:
            self.assertIn(stage, seen)
        # The first pass follows the real operation order; later re-entry
        # across routes/rounds is allowed and is not constrained here.
        first_order = [first_seen[stage] for stage in expected]
        self.assertEqual(first_order, sorted(first_order))
        self.assertEqual(len(set(first_order)), len(first_order))
        self.assertNotIn("PUBLISHED", seen)
        self.assertEqual(self.case["result"].status, "SUCCESS")

    def test_corridor_evidence_reaches_progress_and_result(self):
        events = self.case["progress_events"]
        completes = [
            event for event in events
            if event["stage"] == "NLP_SOLVE"
            and event["metrics"].get("phase") == "complete"]
        self.assertTrue(completes)
        last = completes[-1]["metrics"]
        count = last["corridor_halfspace_constraint_count"]
        residual = last["corridor_max_residual_m"]
        self.assertIsInstance(count, int)
        self.assertGreater(count, 0)
        self.assertIsNotNone(residual)
        self.assertGreaterEqual(float(residual), 0.0)
        self.assertLessEqual(float(residual), 1e-8)
        result = self.case["result"]
        self.assertEqual(
            result.metrics["corridor_halfspace_constraint_count"], count)
        self.assertEqual(result.metrics["corridor_max_residual_m"],
                         float(residual))
        last_round = result.search["rounds"][-1]
        self.assertEqual(
            last_round["corridor_halfspace_constraint_count"], count)
        self.assertEqual(
            last_round["corridor_max_residual_m"], float(residual))

    def test_callback_exception_is_recorded_and_disables_later_events(self):
        calls = []

        def failing(event):
            calls.append("call")
            raise RuntimeError("display callback failure")

        emitter = pipeline_module._ProgressEmitter(
            self.case["map_request"], failing)
        emitter.emit("ASTAR_SEARCH", metrics={"phase": "running"})
        self.assertEqual(len(calls), 1)
        self.assertEqual(emitter.error_count, 1)
        self.assertIn("display callback failure", emitter.error)
        emitter.emit("NLP_SOLVE", metrics={"phase": "running"})
        self.assertEqual(len(calls), 1)


class _RaisingPublisher(object):

    def __init__(self):
        self.calls = 0

    def publish(self, message):
        self.calls += 1
        raise RuntimeError("publisher failure")


class ProgressiveDocumentTest(unittest.TestCase):
    """Node-level progressive documents: identity, sampling and freeze."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()
        cls.request = cls.case["map_request"].request
        cls.snapshot = g5.snapshot_from_request(cls.request)
        cls.plan_id = cls.case["identity"]["plan_id"]
        cls.generation = 1

    def _node(self):
        node = object.__new__(_PLANNER_NODE.UniformPlannerNode)
        node.backend = "virtual_rigid_a"
        node.planning_debug_pub = _Publisher()
        node._lock = threading.RLock()
        node._generation = self.generation
        node._state = "PLANNING"
        node._state_plan = self.plan_id
        node._virtual_snapshot = self.snapshot
        node._virtual_request = self.request
        node._backend_hooks = object()
        node._virtual_admission = {
            "formation_id": self.snapshot.formation_id,
            "request_sha256": self.case["map_request"].request_sha256,
            "grid_sha256": self.case["result"].grid_sha256,
            "planning_context_sha256":
                self.case["result"].planning_context_sha256,
        }
        node._planning_progress = None
        node._progress_layers = {}
        node._progress_display_errors = 0
        node._progress_display_disabled = set()
        return node

    def _astar(self):
        return [{"route_id": 0,
                 "points": [[float(point[0]), float(point[1])]
                            for point in self.case["route"].points]}]

    def _event(self, **overrides):
        event = {
            "schema": PROGRESS_SCHEMA,
            "stage": "PORTAL_GUIDE",
            "stage_index": PROGRESS_STAGE_INDEX["PORTAL_GUIDE"],
            "state": "PLANNING",
            "elapsed_s": 1.25,
            "route_index": 0,
            "route_count": 1,
            "round_index": 0,
            "request_sha256": self.case["map_request"].request_sha256,
            "grid_sha256": self.case["result"].grid_sha256,
            "planning_context_sha256":
                self.case["result"].planning_context_sha256,
            "metrics": {"phase": "running"},
            "diagnostic_path": self.case["result"].path,
            "diagnostic_display": {
                "astar": self._astar(),
                "result": self.case["result"],
            },
        }
        event.update(overrides)
        return event

    def _documents(self, node):
        return [json.loads(message.data)
                for message in node.planning_debug_pub.messages]

    def test_real_event_creates_all_three_progressive_layers(self):
        node = self._node()
        node._on_virtual_progress(
            self.generation, self.plan_id, self.snapshot, self._event())
        documents = self._documents(node)
        self.assertEqual(len(documents), 1)
        document = documents[0]
        self.assertEqual(document["schema"], PLANNING_DEBUG_SCHEMA)
        self.assertEqual(document["state"], PLANNING_STATE)
        self.assertEqual(document["generation"], self.generation)
        self.assertEqual(document["model_sha256"], None)
        self.assertEqual(document["astar"], self._astar())
        self.assertEqual(len(document["corridors"]), 1)
        candidates = document["candidates"]
        self.assertEqual(candidates["schema"], CANDIDATE_SCHEMA)
        self.assertEqual(candidates["classification"], CANDIDATE_UNVERIFIED)
        self.assertFalse(candidates["certified"])
        self.assertEqual(candidates["diagnostic_label"],
                         CANDIDATE_DIAGNOSTIC_LABEL)

    def test_wrong_identity_or_generation_publishes_nothing(self):
        variants = (
            ("request_sha256", "0" * 64),
            ("grid_sha256", "1" * 64),
            ("planning_context_sha256", "2" * 64),
            ("schema", "virtual_a_progress/v0"),
            ("state", "PUBLISHED"),
            ("stage", "PUBLISHED"),
            ("stage_index", 99),
        )
        for name, value in variants:
            with self.subTest(name=name):
                node = self._node()
                node._on_virtual_progress(
                    self.generation, self.plan_id, self.snapshot,
                    self._event(**{name: value}))
                self.assertEqual(node.planning_debug_pub.messages, [])
        node = self._node()
        node._on_virtual_progress(
            self.generation + 1, self.plan_id, self.snapshot, self._event())
        self.assertEqual(node.planning_debug_pub.messages, [])
        node = self._node()
        node._on_virtual_progress(
            self.generation, self.plan_id + 1, self.snapshot, self._event())
        self.assertEqual(node.planning_debug_pub.messages, [])
        node = self._node()
        node._on_virtual_progress(
            self.generation, self.plan_id, object(), self._event())
        self.assertEqual(node.planning_debug_pub.messages, [])

    def test_failed_document_freezes_completed_layers(self):
        node = self._node()
        node._on_virtual_progress(
            self.generation, self.plan_id, self.snapshot, self._event())
        node._publish_planning_failed(
            self.generation, self.plan_id, self.snapshot,
            self.case["result"], "DYNAMICS_REJECTED", "test failure",
            "dynamics", 2.5)
        documents = self._documents(node)
        self.assertEqual(len(documents), 2)
        failed = documents[-1]
        self.assertEqual(failed["state"], "FAILED")
        self.assertEqual(failed["stage"], PROGRESS_FAILED_STAGE)
        self.assertEqual(failed["stage_index"], PROGRESS_STAGE_INDEX[
            "PORTAL_GUIDE"])
        self.assertEqual(failed["astar"], self._astar())
        self.assertEqual(len(failed["corridors"]), 1)
        self.assertEqual(failed["candidates"]["classification"],
                         CANDIDATE_UNVERIFIED)
        self.assertEqual(failed["reason"], "DYNAMICS_REJECTED")

    def test_candidate_is_pointwise_from_the_real_state_at_s(self):
        node = self._node()
        node._on_virtual_progress(
            self.generation, self.plan_id, self.snapshot, self._event())
        candidates = self._documents(node)[0]["candidates"]
        path = self.case["result"].path
        length = float(path.length_m)
        count = min(
            CANDIDATE_POINT_LIMIT,
            max(2, int(math.ceil(length / CANDIDATE_SAMPLE_SPACING_M)) + 1))
        stations = [length * index / float(count - 1)
                    for index in range(count)]
        self.assertEqual(len(candidates["f_points"]), count)
        centers = {vehicle_id: []
                   for vehicle_id in (
                       vehicle["vehicle_id"]
                       for vehicle in candidates["vehicles"])}
        for index, station in enumerate(stations):
            state = path.state_at_s(station)
            qf = state.qf
            got = candidates["f_points"][index]
            self.assertLessEqual(abs(got[0] - qf.x), 1e-12)
            self.assertLessEqual(abs(got[1] - qf.y), 1e-12)
            for vehicle in state.vehicles:
                centers[vehicle.vehicle_id].append(
                    (vehicle.x, vehicle.y))
                payload = next(
                    item for item in candidates["vehicles"]
                    if item["vehicle_id"] == vehicle.vehicle_id)
                point = payload["center_points"][index]
                self.assertLessEqual(abs(point[0] - vehicle.x), 1e-12)
                self.assertLessEqual(abs(point[1] - vehicle.y), 1e-12)
        ids = sorted(centers)
        for first in range(len(ids)):
            for second in range(first + 1, len(ids)):
                distances = [
                    math.hypot(a[0] - b[0], a[1] - b[1])
                    for a, b in zip(centers[ids[first]], centers[ids[second]])]
                self.assertLessEqual(
                    max(distances) - min(distances), 1e-10)
        # OBB corners use each vehicle's actual body heading at that station.
        obb_count = min(CANDIDATE_OBB_LIMIT, count)
        if obb_count == 1:
            obb_indices = [0]
        else:
            obb_indices = sorted({
                int(round(index * (count - 1) / float(obb_count - 1)))
                for index in range(obb_count)})
        shapes = self.request.vehicle_shapes
        for vehicle in candidates["vehicles"]:
            vehicle_id = vehicle["vehicle_id"]
            shape = shapes[vehicle_id]
            half_length = 0.5 * shape.length_m
            half_width = 0.5 * shape.width_m
            self.assertEqual(len(vehicle["obb_corners"]), len(obb_indices))
            for position, station_index in enumerate(obb_indices):
                state = path.state_at_s(stations[station_index])
                jet = next(item for item in state.vehicles
                           if item.vehicle_id == vehicle_id)
                cosine, sine = math.cos(jet.theta), math.sin(jet.theta)
                expected = (
                    jet.x + cosine * half_length - sine * half_width,
                    jet.y + sine * half_length + cosine * half_width)
                outline = vehicle["obb_corners"][position]
                self.assertEqual(len(outline), 5)
                self.assertEqual(outline[0], outline[-1])
                self.assertLessEqual(abs(outline[0][0] - expected[0]), 1e-12)
                self.assertLessEqual(abs(outline[0][1] - expected[1]), 1e-12)

    def test_publisher_failure_disables_later_progress_only(self):
        node = self._node()
        node.planning_debug_pub = _RaisingPublisher()
        with mock.patch.object(_PLANNER_NODE.rospy, "logerr"):
            node._on_virtual_progress(
                self.generation, self.plan_id, self.snapshot, self._event())
        self.assertIn(self.generation, node._progress_display_disabled)
        self.assertEqual(node._progress_display_errors, 1)
        node.planning_debug_pub = _Publisher()
        node._on_virtual_progress(
            self.generation, self.plan_id, self.snapshot, self._event())
        self.assertEqual(node.planning_debug_pub.messages, [])


class ProgressiveLayerLifecycleTest(unittest.TestCase):
    """Only a new admission or CLEARED empties the diagnostic layers."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()
        cls.plan_id = cls.case["identity"]["plan_id"]
        cls.identity = cls.case["identity"]
        cls.candidate = sample_candidate_path(
            cls.case["result"].path,
            cls.case["map_request"].request.vehicle_shapes,
            source="test", status=cls.case["result"].status)

    def _node(self):
        node = object.__new__(_NODE.FormationPlaybackNode)
        node._lock = threading.RLock()
        node.frame_id = "map"
        node.backend = "virtual_rigid_a"
        node.transport_ui = False
        node._job_state = "PLAYING"
        node._job_state_plan = self.plan_id
        job = dict(self.case["job"])
        job["request_sha256"] = (
            self.case["map_request"].request_sha256)
        node._virtual_jobs = {self.plan_id: job}
        node._raw_requests = {}
        node._planning_progress = None
        node.trajectory = None
        node.hold = None
        node._hold_reason = "START"
        node._active_plan = None
        node._previewing = False
        node._playback_model_sha256 = None
        node._playback_semantic_version = None
        node._clear_markers_pending = False
        node._last_state = None
        node.corridor_pub = _Publisher()
        node.guide_pub = _Publisher()
        node.candidate_pub = _Publisher()
        node.guide_path_pubs = {
            "/formation/path/astar_raw_{}".format(route_id): _Publisher()
            for route_id in range(3)}
        node.guide_path_pubs.update({
            "/formation/path/portal_guide_{}".format(route_id): _Publisher()
            for route_id in range(3)})
        node.guide_path_pubs.update({
            "/formation/path/astar_active": _Publisher(),
            "/formation/path/portal_guide_active": _Publisher()})
        node.candidate_path_pubs = {
            "/formation/path/f_candidate": _Publisher(),
            **{"/formation/path/candidate_vehicle_{}".format(vehicle_id):
               _Publisher() for vehicle_id in (1, 2, 3)},
        }
        node.status_pub = _Publisher()
        node.marker_pub = _Publisher()
        node.hud_pub = None
        return node

    def _planning_document(self, **overrides):
        corridor = copy.deepcopy(self.case["display"])
        corridor["model_sha256"] = None
        document = {
            "schema": PLANNING_DEBUG_SCHEMA,
            "state": PLANNING_STATE,
            "plan_id": self.identity["plan_id"],
            "formation_id": self.identity["formation_id"],
            "backend": self.identity["backend"],
            "generation": self.identity["generation"],
            "request_sha256": self.case["map_request"].request_sha256,
            "grid_sha256": self.identity["grid_sha256"],
            "planning_context_sha256":
                self.identity["planning_context_sha256"],
            "model_sha256": None,
            "stage": "PORTAL_GUIDE", "stage_index": 3,
            "elapsed_s": 1.0, "route_index": 0, "route_count": 1,
            "round_index": 0, "reason": "", "detail": "", "metrics": {},
            "astar": self._astar(),
            "corridors": [corridor],
        }
        document.update(overrides)
        return document

    def _astar(self):
        return [{"route_id": 0,
                 "points": [[float(point[0]), float(point[1])]
                            for point in self.case["route"].points]}]

    def _publish(self, node, document):
        with mock.patch.object(
                _NODE.rospy.Time, "now",
                return_value=_NODE.rospy.Time(0, 0)):
            node.on_planning_debug(SimpleNamespace(data=json.dumps(document)))

    def _job_state(self, node, report):
        with mock.patch.object(
                _NODE.rospy.Time, "now",
                return_value=_NODE.rospy.Time(0, 0)):
            node.on_job_state(SimpleNamespace(data=json.dumps(report)))

    def _report(self, state, plan_id):
        report = dict(self.case["job"])
        report.update({
            "state": state, "plan_id": plan_id,
            "formation_id": self.identity["formation_id"],
            "backend": "virtual_rigid_a",
            "request_sha256": self.case["map_request"].request_sha256,
        })
        return report

    def test_new_planning_admission_clears_all_three_layers(self):
        node = self._node()
        self._job_state(node, self._report("PLANNING", self.plan_id + 1))
        for publisher in (node.corridor_pub, node.guide_pub,
                          node.candidate_pub):
            self.assertEqual(len(publisher.messages), 1)
            self.assertEqual(publisher.messages[0].markers[0].action,
                             Marker.DELETEALL)
        self.assertIsNone(node._planning_progress)
        for registry in (node.guide_path_pubs, node.candidate_path_pubs):
            self.assertTrue(registry)
            for publisher in registry.values():
                self.assertEqual(len(publisher.messages), 1)
                self.assertFalse(publisher.messages[0].poses)

    def test_heartbeat_failed_and_raw_request_never_clear_layers(self):
        node = self._node()
        self._job_state(node, self._report("PLANNING", self.plan_id))
        self._job_state(node, self._report("FAILED", self.plan_id))
        request = SimpleNamespace(
            plan_id=self.plan_id,
            formation_id=self.identity["formation_id"],
            formation_start=SimpleNamespace(x=0.0, y=0.0, theta=0.0),
            formation_goal=SimpleNamespace(x=1.0, y=0.0, theta=0.0),
            allow_reverse=False,
            vehicles=[
                SimpleNamespace(
                    vehicle_id=vehicle_id,
                    offset_in_formation=SimpleNamespace(x=0.0, y=0.0),
                    start_center=SimpleNamespace(x=0.0, y=0.0, theta=0.0),
                    goal_center=SimpleNamespace(x=1.0, y=0.0, theta=0.0),
                    goal_heading_free=False,
                    body_length_m=0.7, body_width_m=0.5)
                for vehicle_id in (1, 2, 3)])
        node.on_request(request)
        for publisher in (node.corridor_pub, node.guide_pub,
                          node.candidate_pub):
            self.assertEqual(publisher.messages, [])
        self.assertIsNone(node.trajectory)
        self.assertIn(self.plan_id, node._raw_requests)
        with mock.patch.object(
                _NODE.rospy.Time, "now",
                return_value=_NODE.rospy.Time(0, 0)):
            node._clear("test")
        for publisher in (node.corridor_pub, node.guide_pub,
                          node.candidate_pub):
            self.assertEqual(publisher.messages, [])

    def test_astar_only_document_publishes_orange_and_spares_corridor(self):
        node = self._node()
        astar_document = self._planning_document()
        astar_document.pop("corridors")
        self._publish(node, astar_document)
        self.assertEqual(node.corridor_pub.messages, [])
        self.assertEqual(len(node.guide_pub.messages), 1)
        markers = node.guide_pub.messages[0].markers
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertFalse(any(marker.type in (Marker.LINE_LIST,
                                             Marker.LINE_STRIP)
                             for marker in markers))
        self.assertTrue(any(marker.ns == PLANNING_ASTAR_NAMESPACE
                            and marker.type == Marker.TEXT_VIEW_FACING
                            for marker in markers))
        astar_pub = node.guide_path_pubs["/formation/path/astar_raw_0"]
        self.assertTrue(astar_pub.messages[-1].poses)
        self.assertEqual(
            len(astar_pub.messages[-1].poses),
            len(self.case["route"].points))
        for route_id in (1, 2):
            self.assertFalse(
                node.guide_path_pubs[
                    "/formation/path/astar_raw_{}".format(route_id)
                ].messages[-1].poses)
        self.assertFalse(
            node.guide_path_pubs["/formation/path/portal_guide_0"]
            .messages[-1].poses)

    def test_active_aliases_follow_real_route_index(self):
        node = self._node()
        document = self._planning_document()
        document.pop("corridors")
        document["astar"] = [
            {"route_id": 0, "points": [[0.0, 0.0], [0.0, 1.0]]},
            {"route_id": 1, "points": [[1.0, 0.0], [1.0, 1.0]]},
            {"route_id": 2, "points": [[2.0, 0.0], [2.0, 1.0]]},
        ]
        document.update({"route_index": 2, "route_count": 3})
        self._publish(node, document)
        active_astar = node.guide_path_pubs[
            "/formation/path/astar_active"].messages[-1]
        self.assertEqual(
            [(pose.pose.position.x, pose.pose.position.y)
             for pose in active_astar.poses],
            [(2.0, 0.0), (2.0, 1.0)])
        self.assertFalse(node.guide_path_pubs[
            "/formation/path/portal_guide_active"].messages[-1].poses)

        changed = copy.deepcopy(document)
        changed.update({"route_index": 1, "route_count": 3})
        self._publish(node, changed)
        astar_pub = node.guide_path_pubs["/formation/path/astar_active"]
        self.assertEqual(
            [(pose.pose.position.x, pose.pose.position.y)
             for pose in astar_pub.messages[-1].poses],
            [(1.0, 0.0), (1.0, 1.0)])
        self.assertFalse(astar_pub.messages[-2].poses)

        invalid = copy.deepcopy(changed)
        invalid["route_index"] = 3
        self._publish(node, invalid)
        self.assertFalse(astar_pub.messages[-1].poses)
        self.assertFalse(node.guide_path_pubs[
            "/formation/path/portal_guide_active"].messages[-1].poses)

    def test_active_alias_never_falls_back_to_route_zero(self):
        node = self._node()
        document = self._planning_document()
        document.pop("corridors")
        document["astar"] = [
            {"route_id": 0, "points": [[0.0, 0.0], [0.0, 1.0]]},
        ]
        document.update({"route_index": 2, "route_count": 3})
        self._publish(node, document)
        self.assertFalse(node.guide_path_pubs[
            "/formation/path/astar_active"].messages[-1].poses)
        self.assertTrue(node.guide_path_pubs[
            "/formation/path/astar_raw_0"].messages[-1].poses)

    def test_active_guide_follows_corridor_route_id(self):
        node = self._node()
        corridors = []
        for route_id in range(3):
            corridor = copy.deepcopy(self.case["display"])
            corridor["model_sha256"] = None
            corridor["route_id"] = route_id
            corridor["raw_route_points"] = [
                [float(route_id), 0.0], [float(route_id), 1.0]]
            corridor["guide_points"] = [
                [float(route_id), 0.5], [float(route_id), 1.5]]
            corridors.append(corridor)
        document = self._planning_document(
            corridors=corridors, route_index=2, route_count=3)
        self._publish(node, document)
        active_astar = node.guide_path_pubs[
            "/formation/path/astar_active"].messages[-1]
        active_guide = node.guide_path_pubs[
            "/formation/path/portal_guide_active"].messages[-1]
        self.assertEqual(
            [(pose.pose.position.x, pose.pose.position.y)
             for pose in active_astar.poses],
            [(2.0, 0.0), (2.0, 1.0)])
        self.assertEqual(
            [(pose.pose.position.x, pose.pose.position.y)
             for pose in active_guide.poses],
            [(2.0, 0.5), (2.0, 1.5)])
        for route_id in range(3):
            self.assertTrue(node.guide_path_pubs[
                "/formation/path/astar_raw_{}".format(route_id)
            ].messages[-1].poses)

    def test_candidate_payload_publishes_paths_and_keeps_obb_markers(self):
        node = self._node()
        self._publish(
            node, self._planning_document(candidates=self.candidate))
        for topic in ("/formation/path/f_candidate",
                      "/formation/path/candidate_vehicle_1",
                      "/formation/path/candidate_vehicle_2",
                      "/formation/path/candidate_vehicle_3"):
            self.assertTrue(node.candidate_path_pubs[topic].messages[-1].poses)
        markers = node.candidate_pub.messages[-1].markers
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertFalse(any(
            marker.type == Marker.LINE_STRIP
            and (marker.ns == CANDIDATE_F_NAMESPACE
                 or marker.ns.endswith("_center_candidate"))
            for marker in markers))
        self.assertTrue(any(
            marker.type == Marker.LINE_STRIP
            and marker.ns.endswith("_obb_candidate")
            for marker in markers))
        self.assertTrue(
            node.guide_path_pubs["/formation/path/astar_raw_0"]
            .messages[-1].poses)
        self.assertTrue(
            node.guide_path_pubs["/formation/path/portal_guide_0"]
            .messages[-1].poses)

    def test_null_corridors_clears_corridor_and_repaints_astar(self):
        node = self._node()
        self._publish(node, self._planning_document())
        self.assertEqual(len(node.corridor_pub.messages), 1)
        self.assertEqual(len(node.guide_pub.messages), 1)
        self._publish(node, self._planning_document(corridors=None))
        self.assertEqual(len(node.corridor_pub.messages), 2)
        self.assertEqual(node.corridor_pub.messages[1].markers[0].action,
                         Marker.DELETEALL)
        self.assertEqual(len(node.guide_pub.messages), 2)
        markers = node.guide_pub.messages[1].markers
        self.assertFalse(any(
            marker.type in (Marker.LINE_LIST, Marker.LINE_STRIP)
            for marker in markers))
        self.assertTrue(any(
            marker.ns == PLANNING_ASTAR_NAMESPACE
            and marker.type == Marker.TEXT_VIEW_FACING
            for marker in markers))
        self.assertTrue(
            node.guide_path_pubs["/formation/path/astar_raw_0"]
            .messages[-1].poses)
        self.assertFalse(
            node.guide_path_pubs["/formation/path/portal_guide_0"]
            .messages[-1].poses)

    def test_null_candidates_with_failure_draws_only_the_failure_layer(self):
        node = self._node()
        document = self._planning_document(candidates=None)
        document["failure"] = {
            "x": 1.0, "y": 2.0, "reason": "COLLISION", "source": "COLLISION"}
        self._publish(node, document)
        self.assertEqual(len(node.candidate_pub.messages), 1)
        markers = node.candidate_pub.messages[0].markers
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        failure = [marker for marker in markers
                   if marker.ns == PLANNING_FAILURE_NAMESPACE]
        self.assertTrue(any(marker.type == Marker.SPHERE
                            for marker in failure))
        self.assertFalse(any(marker.ns == CANDIDATE_F_NAMESPACE
                             for marker in markers))
        self.assertTrue(node.candidate_path_pubs)
        for publisher in node.candidate_path_pubs.values():
            self.assertEqual(len(publisher.messages), 1)
            self.assertFalse(publisher.messages[0].poses)

    def test_null_candidates_without_failure_clears_candidates(self):
        node = self._node()
        document = self._planning_document(candidates=None)
        self._publish(node, document)
        self.assertEqual(len(node.candidate_pub.messages), 1)
        markers = node.candidate_pub.messages[0].markers
        self.assertEqual([marker.action for marker in markers],
                         [Marker.DELETEALL])
        for publisher in node.candidate_path_pubs.values():
            self.assertEqual(len(publisher.messages), 1)
            self.assertFalse(publisher.messages[0].poses)

    def test_progress_record_keeps_the_planning_formation_and_elapsed(self):
        node = self._node()
        self._publish(node, self._planning_document())
        self.assertEqual(node._planning_progress["formation_id"],
                         self.identity["formation_id"])
        with mock.patch.object(
                _NODE.rospy.Time, "now",
                return_value=_NODE.rospy.Time(10, 0)):
            view = node._hud_progress()
        self.assertEqual(view["formation_id"],
                         self.identity["formation_id"])
        self.assertAlmostEqual(view["elapsed_s"], 11.0, places=6)


class CandidateMarkerTest(unittest.TestCase):
    """Candidate markers copy the sampled payload; no certification early."""

    @classmethod
    def setUpClass(cls):
        cls.case = _actual_corridor_case()
        cls.payload = sample_candidate_path(
            cls.case["result"].path,
            cls.case["map_request"].request.vehicle_shapes,
            source="test", status=cls.case["result"].status)

    def test_unverified_candidate_uses_stable_namespaces_and_labels(self):
        document = {"state": PLANNING_STATE, "candidates": self.payload}
        markers, paths = build_candidate_display(document)
        self.assertEqual(markers[0].action, "DELETEALL")
        namespaces = {marker.namespace for marker in markers}
        self.assertIn(CANDIDATE_F_NAMESPACE, namespaces)
        for vehicle_id in (1, 2, 3):
            self.assertIn(CANDIDATE_CENTER_NAMESPACE.format(vehicle_id),
                          namespaces)
            self.assertIn(CANDIDATE_OBB_NAMESPACE.format(vehicle_id),
                          namespaces)
        self.assertLessEqual(len(namespaces), 9)
        f_path = paths["f_candidate"]
        self.assertEqual(len(f_path), len(self.payload["f_points"]))
        for got, want in zip(f_path, self.payload["f_points"]):
            self.assertLessEqual(abs(got[0] - want[0]), 1e-12)
            self.assertLessEqual(abs(got[1] - want[1]), 1e-12)
        for vehicle in self.payload["vehicles"]:
            vehicle_id = vehicle["vehicle_id"]
            centre = paths[("candidate_vehicle", vehicle_id)]
            self.assertEqual(len(centre), len(vehicle["center_points"]))
            for got, want in zip(centre, vehicle["center_points"]):
                self.assertLessEqual(abs(got[0] - want[0]), 1e-12)
                self.assertLessEqual(abs(got[1] - want[1]), 1e-12)
        self.assertFalse(any(
            marker.marker_type == "LINE_STRIP"
            and (marker.namespace == CANDIDATE_F_NAMESPACE
                 or marker.namespace.endswith("_center_candidate"))
            for marker in markers))
        obb_lines = [
            marker for marker in markers
            if marker.namespace.endswith("_obb_candidate")]
        self.assertEqual(CANDIDATE_OBB_LIMIT, 5)
        self.assertLessEqual(len(obb_lines), 3 * CANDIDATE_OBB_LIMIT)
        for vehicle in self.payload["vehicles"]:
            self.assertLessEqual(
                len(vehicle["obb_corners"]), CANDIDATE_OBB_LIMIT)
        self.assertTrue(all(marker.color == CANDIDATE_OBB_COLOR
                            for marker in obb_lines))
        self.assertTrue(all(marker.scale[0] == 0.010
                            for marker in obb_lines))
        texts = [marker for marker in markers
                 if marker.marker_type == "TEXT_VIEW_FACING"]
        self.assertTrue(any(CANDIDATE_DIAGNOSTIC_LABEL in marker.text
                            for marker in texts))
        self.assertFalse(any("CERTIFIED" in marker.text
                             for marker in texts))
        f_label = next(marker for marker in texts
                       if marker.namespace == CANDIDATE_F_NAMESPACE)
        self.assertEqual(f_label.scale[2], 0.14)

    def test_certified_payload_requires_a_published_document(self):
        certified = sample_candidate_path(
            self.case["result"].path,
            self.case["map_request"].request.vehicle_shapes,
            classification=CANDIDATE_CERTIFIED,
            source="test", status=self.case["result"].status)
        with self.assertRaises(PlaybackValidationError):
            build_candidate_display(
                {"state": PLANNING_STATE, "candidates": certified})
        markers = build_candidate_display(
            {"state": PLANNING_PUBLISHED_STATE,
             "candidates": certified})[0]
        texts = [marker.text for marker in markers
                 if marker.marker_type == "TEXT_VIEW_FACING"]
        self.assertTrue(any("CERTIFIED" in text for text in texts))

    def test_failure_markers_need_a_real_finite_coordinate(self):
        document = {
            "state": "FAILED",
            "candidates": None,
            "failure": {"x": 1.5, "y": -2.0, "reason": "COLLISION",
                        "source": "COLLISION"},
        }
        markers, paths = build_candidate_display(document)
        self.assertEqual(paths, {})
        self.assertEqual(markers[0].action, "DELETEALL")
        failure = [marker for marker in markers
                   if marker.namespace == PLANNING_FAILURE_NAMESPACE]
        self.assertTrue(any(marker.marker_type == "SPHERE"
                            for marker in failure))
        self.assertTrue(any(marker.marker_type == "TEXT_VIEW_FACING"
                            and "COLLISION" in marker.text
                            for marker in failure))
        self.assertFalse(any(
            marker.namespace == CANDIDATE_F_NAMESPACE
            for marker in markers))
        with self.assertRaises(PlaybackValidationError):
            build_candidate_display({
                "state": "FAILED", "candidates": None,
                "failure": {"x": float("nan"), "y": 0.0,
                            "reason": "COLLISION", "source": "COLLISION"},
            })


if __name__ == "__main__":
    unittest.main()