"""FR03 S01--S09: finite-label states, keys, successors and proxy cost.

Scope is exactly the FR03 contract: exact states with ``q_F``, exact body
headings, mode and previous wheel/primitive signature; a finite deduplication
key (0.20 m XY, 10 deg psi/heading, mode and canonical wheel labels); O/A/S
successors from the fixed library; the real analytic-pose OBB sweep; and a
seconds-dimension proxy cost made of the actual FR02 setup demand plus a
first-order motion estimate.  No whole-map search, ROS or NLP is implemented.

Every accepted successor class is additionally submitted to the FR02 audit
(``UniformTimedPlan.from_edges`` + ``audit_uniform_timed_plan``), which is the
integration point for FR04.  The search itself never runs full S8/S9 per
expansion; S08 proves that by patching both entry points.

Evidence is written to ``FR03/successors.json`` (override with
``FR03_RESULTS_PATH``) and the ``FR03`` artifact directory (override with
``FR03_ARTIFACT_DIR``): key/state samples, successor-to-FR02 audit results and
the fixed-seed 200-state local performance measurement.

Run from the V0.2 worktree root::

    FR03_RESULTS_PATH=evaluation/formation_refactor_v02/FR03/successors.json \\
    FR03_ARTIFACT_DIR=evaluation/formation_refactor_v02/FR03 \\
    OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests \\
        -p 'test_uniform_search.py' -v
"""
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import statistics
import sys
import tracemalloc
import unittest

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import (
    FormationCollisionChecker,
    SweepPolicy,
    VehicleShape,
)
from fwis_v3_optimizer.formation_quality import FEASIBLE, REJECTED
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.uniform_motion import (
    MODE_A,
    MODE_O,
    MODE_S,
    a_arc_edge,
    a_arc_required_headings,
    a_straight_edge,
    a_straight_required_headings,
    o_translation_edge,
    s_spin_edge,
)
from fwis_v3_optimizer.uniform_quality import (
    UNIFORM_DYNAMIC_UNCONFIRMED,
    UniformAuditRequest,
    audit_uniform_timed_plan,
)
from fwis_v3_optimizer.uniform_search import (
    RepresentativeTable,
    SuccessorConfig,
    SuccessorGenerator,
    UniformSearchState,
    analytic_sweep_segment,
    continuity_error,
    initial_state,
    initial_wheel_signature,
    make_key,
    motion_proxy_s,
    setup_demand_s,
    wheel_signature_from_edge,
)
from fwis_v3_optimizer.uniform_timing import timing_motion_limits
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan

OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
BODY_SHAPES = {vehicle_id: VehicleShape(0.738, 0.500)
               for vehicle_id, _, _ in OFFSETS}
VEHICLE_IDS = tuple(vehicle_id for vehicle_id, _, _ in OFFSETS)
MAP_RES = 0.05
MAP_ORIGIN = -15.0

RESULTS_PATH = Path(os.environ.get(
    "FR03_RESULTS_PATH",
    str(REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR03"
        / "successors.json")))
ARTIFACT_DIR = Path(os.environ.get(
    "FR03_ARTIFACT_DIR",
    str(REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR03")))
RESULTS = []

SOURCE_FILES = (
    "uniform_search.py",
    "uniform_motion.py",
    "uniform_timing.py",
    "uniform_trajectory.py",
    "uniform_quality.py",
    "formation_collision.py",
)
CONTINUITY_TOL = 1e-7


def model():
    return FormationModel(tuple(
        VehicleOffset(vehicle_id, x, y) for vehicle_id, x, y in OFFSETS))


def headings(value=0.0):
    return {vehicle_id: float(value) for vehicle_id in VEHICLE_IDS}


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def directions(value=1):
    return {vehicle_id: int(value) for vehicle_id in VEHICLE_IDS}


def empty_grid():
    return Grid(np.zeros((600, 600), dtype=bool), MAP_RES, MAP_ORIGIN,
                MAP_ORIGIN)


def grid_with(points):
    occ = np.zeros((600, 600), dtype=bool)
    for x, y in points:
        row = int(math.floor((y - MAP_ORIGIN) / MAP_RES))
        col = int(math.floor((x - MAP_ORIGIN) / MAP_RES))
        occ[row, col] = True
    return Grid(occ, MAP_RES, MAP_ORIGIN, MAP_ORIGIN)


def checker(grid, mdl=None):
    return FormationCollisionChecker(
        grid, mdl or model(), BODY_SHAPES, 0.05)


def config(**overrides):
    values = dict(timing_limits=FormationTimingLimits())
    values.update(overrides)
    return SuccessorConfig(**values)


def generator(grid=None, cfg=None):
    cfg = cfg or config()
    return SuccessorGenerator(model(), checker(grid or empty_grid()), cfg)


def state_from_edge(edge):
    return UniformSearchState(
        qf=edge.end_qf, headings=tuple(edge.end_headings), mode=edge.mode,
        wheel=wheel_signature_from_edge(edge), g=0.0)


def find_successor(result, primitive=None, label=None, label_prefix=None,
                   end_headings=None):
    for item in result.successors:
        if primitive is not None and item.edge.primitive != primitive:
            continue
        if label is not None and item.label != label:
            continue
        if label_prefix is not None and not item.label.startswith(label_prefix):
            continue
        if end_headings is not None:
            actual = dict(item.edge.end_headings)
            if not all(
                    abs(math.atan2(math.sin(actual[vehicle_id] - theta),
                                   math.cos(actual[vehicle_id] - theta))) < 1e-9
                    for vehicle_id, theta in end_headings.items()):
                continue
        return item
    return None


def rejection_reasons(result):
    return [item["reason"] for item in result.rejections]


def audit_edges(edges, grid=None, mdl=None):
    mdl = mdl or model()
    plan = UniformTimedPlan.from_edges(tuple(edges), BODY_SHAPES)
    projection = plan.sample()
    result = audit_uniform_timed_plan(
        UniformAuditRequest.from_plan(plan), plan, projection,
        grid or empty_grid())
    failure = getattr(result, "failure", None)
    return {
        "status": str(result.status),
        "reason": str(result.reason),
        "feasible": bool(result.feasible),
        "executable": bool(getattr(result, "executable", False)),
        "duration_s": float(plan.duration_s),
        "failure_code": getattr(failure, "code", None),
        "failure_constraint": getattr(failure, "constraint", None),
        "failure_vehicle_id": getattr(failure, "vehicle_id", None),
        "failure_detail": getattr(failure, "detail", None),
        "dynamic_continuous_certificate": result.metrics.get(
            "dynamic_confirmation", {}).get(
                "all_constraints_confirmed"),
    }


def exact_peaks(plan, samples=20001):
    """Exact analytic-model peaks over a fine time grid (not the S9 grid)."""
    peaks = {"speed": 0.0, "acceleration": 0.0, "jerk": 0.0}
    for index in range(samples):
        time_s = plan.duration_s * index / float(samples - 1)
        state = plan.state_at(time_s)
        for vehicle in state.vehicles:
            peaks["speed"] = max(peaks["speed"],
                                 abs(vehicle.center_speed_mps))
            peaks["acceleration"] = max(
                peaks["acceleration"], vehicle.center_acceleration_mps2)
            peaks["jerk"] = max(peaks["jerk"], vehicle.center_jerk_mps3)
    return peaks


@contextmanager
def record_case(case_id, title, inputs):
    record = {"case": case_id, "title": title, "status": "FAILED",
              "inputs": inputs, "subcases": []}
    RESULTS.append(record)
    try:
        yield record
    except Exception as exc:
        record["status"] = "FAILED" if isinstance(
            exc, AssertionError) else "ERROR"
        record["error_type"] = type(exc).__name__
        record["error"] = str(exc)
        raise
    else:
        record["status"] = "PASSED"


def passed(record, subcase, **values):
    record["subcases"].append(
        dict(name=subcase, status="PASSED", **values))


def write_json_artifact(name, value):
    path = ARTIFACT_DIR / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        value, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return str(path)


def successor_summary(item):
    return {
        "label": item.label,
        "primitive": item.edge.primitive,
        "mode": item.edge.mode,
        "setup_s": item.setup_s,
        "motion_proxy_s": item.motion_proxy_s,
        "cost_s": item.cost_s,
        "end_qf": [item.edge.end_qf.x, item.edge.end_qf.y,
                   item.edge.end_qf.psi],
        "end_headings": {
            str(vehicle_id): theta
            for vehicle_id, theta in item.edge.end_headings},
        "continuity": continuity_error(item.next_state.parent, item),
    }


class UniformSearchAcceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mdl = model()
        cls.motion_limits = timing_motion_limits(FormationTimingLimits())
        cls.start_qf = FormationPose(0.0, 0.0, 0.0)
        cls.start_headings = headings(0.0)
        cls.cfg = config()
        cls.gen = generator(cfg=cls.cfg)

    # -- S01 ---------------------------------------------------------------
    def test_S01_state_alias_keys_and_exact_representative(self):
        """Same q_F with different headings/mode/wheel must not share a key."""
        with record_case(
                "S01", "state alias: exact state, finite key, representative",
                {"xy_bin_m": self.cfg.xy_bin_m,
                 "psi_bin_rad": self.cfg.psi_bin_rad,
                 "heading_bin_rad": self.cfg.heading_bin_rad}) as record:
            qf = FormationPose(0.437, -0.313, 0.213)
            base = initial_state(
                self.mdl, qf, self.start_headings, config=self.cfg)

            spin = s_spin_edge(
                self.mdl, qf, self.start_headings,
                {vehicle_id: 0.3 for vehicle_id in VEHICLE_IDS},
                limits=self.motion_limits)
            state_s = state_from_edge(spin)

            translation = o_translation_edge(
                self.mdl, qf, self.start_headings, (0.8, 0.0),
                directions(), headings(0.0), limits=self.motion_limits)
            state_o = state_from_edge(translation)

            aligned = UniformSearchState(
                qf=qf,
                headings=tuple((vehicle_id, 0.3)
                               for vehicle_id in VEHICLE_IDS),
                mode=MODE_A, wheel=base.wheel, g=0.0)

            keys = {
                "A_headings_0": make_key(base, self.cfg),
                "A_headings_0.3": make_key(aligned, self.cfg),
                "S_headings_0.3": make_key(state_s, self.cfg),
                "O_headings_0": make_key(state_o, self.cfg),
            }
            self.assertNotEqual(keys["A_headings_0"],
                                keys["A_headings_0.3"])
            self.assertNotEqual(keys["A_headings_0.3"],
                                keys["S_headings_0.3"])
            self.assertNotEqual(keys["A_headings_0"],
                                keys["O_headings_0"])
            # The key bins, but the exact state is not snapped.
            self.assertIsInstance(keys["A_headings_0"].xy_bin[0], int)
            self.assertEqual(base.qf.x, 0.437)
            self.assertEqual(base.qf.psi, 0.213)

            table = RepresentativeTable(self.cfg)
            accepted, key, incumbent = table.offer(base)
            self.assertTrue(accepted)
            self.assertIsNone(incumbent)
            worse = UniformSearchState(
                qf=qf, headings=base.headings, mode=base.mode,
                wheel=base.wheel, g=5.0)
            accepted_worse, same_key, representative = table.offer(worse)
            self.assertFalse(accepted_worse)
            self.assertEqual(same_key, key)
            self.assertEqual(representative.qf.x, qf.x)
            self.assertEqual(representative.qf.psi, qf.psi)
            self.assertEqual(representative.g, 0.0)
            self.assertEqual(len(table), 1)

            # Different heading labels are kept as separate representatives.
            accepted_aligned, _, _ = table.offer(aligned)
            self.assertTrue(accepted_aligned)
            self.assertEqual(len(table), 2)

            write_json_artifact("state_key_samples.json", {
                "schema": "fr03.state_key_samples.v1",
                "bins": {
                    "xy_bin_m": self.cfg.xy_bin_m,
                    "psi_bin_rad": self.cfg.psi_bin_rad,
                    "heading_bin_rad": self.cfg.heading_bin_rad,
                    "control_bin": self.cfg.control_bin,
                },
                "states": {
                    "A_headings_0": base.as_dict(),
                    "A_headings_0.3": aligned.as_dict(),
                    "S_headings_0.3": state_s.as_dict(),
                    "O_headings_0": state_o.as_dict(),
                },
                "keys": {name: value.as_dict()
                         for name, value in keys.items()},
                "representative_after_offers": [
                    item.as_dict() for item in (
                        base, aligned)],
                "note": "the key is a deduplication label only; it is not a "
                        "strict dominance certificate over exact states in "
                        "one bin and no pose/heading is snapped",
            })
            passed(record, "keys_distinguish_heading_mode_wheel",
                   key_labels=sorted(keys),
                   representative_count=len(table))

    # -- S02 ---------------------------------------------------------------
    def test_S02_tangent_blocks_A_and_S_unlocks_it(self):
        """A straight without tangent headings is rejected; after S it passes."""
        with record_case(
                "S02", "tangent: no direct A without S alignment",
                {"goal": [1.0, 0.6, 0.0]}) as record:
            goal = FormationPose(1.0, 0.6, 0.0)
            state = initial_state(
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg)
            result = self.gen.generate(state, goal)
            reasons = rejection_reasons(result)
            self.assertIn("A_HEADING_REQUIRES_S_ALIGNMENT", reasons)
            self.assertFalse(any(
                item.edge.primitive == "A_STRAIGHT"
                for item in result.successors))

            tangent = math.atan2(0.6, 1.0)
            required = a_straight_required_headings(
                self.mdl, (1.0, 0.6), directions())
            self.assertTrue(all(
                abs(math.atan2(math.sin(required[v] - tangent),
                               math.cos(required[v] - tangent))) < 1e-9
                for v in VEHICLE_IDS))
            alignment = find_successor(
                result, primitive="S_SPIN", end_headings=required)
            self.assertIsNotNone(alignment)
            self.assertGreater(alignment.setup_s, 0.0)

            after = self.gen.generate(alignment.next_state, goal)
            straight = find_successor(
                after, primitive="A_STRAIGHT", label="A_straight_goal_gear+++")
            self.assertIsNotNone(straight)
            self.assertGreater(straight.setup_s, 0.0)
            audit = audit_edges([alignment.edge, straight.edge])
            self.assertEqual(audit["status"], FEASIBLE)
            self.assertTrue(audit["feasible"])

            passed(record, "A_rejected_then_S_then_A",
                   direct_A_rejections=reasons.count(
                       "A_HEADING_REQUIRES_S_ALIGNMENT"),
                   alignment=successor_summary(alignment),
                   straight=successor_summary(straight),
                   audit=audit)

    # -- S03 ---------------------------------------------------------------
    def test_S03_same_goal_forward_and_reverse(self):
        """The same goal can be reached with forward and reverse A gears."""
        with record_case(
                "S03", "same goal forward/reverse gear schemes",
                {"goal": [1.0, 0.0, 0.0],
                 "note": "one 1.0 m A step reaches the goal exactly"}) as record:
            goal = FormationPose(1.0, 0.0, 0.0)
            state = initial_state(
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg)
            result = self.gen.generate(state, goal)
            forward = find_successor(
                result, primitive="A_STRAIGHT", label="A_straight_goal_gear+++")
            self.assertIsNotNone(forward)
            self.assertAlmostEqual(forward.edge.end_qf.x, 1.0, places=12)
            self.assertEqual(dict(forward.edge.directions), directions(1))

            reverse_required = a_straight_required_headings(
                self.mdl, (1.0, 0.0), directions(-1))
            alignment = find_successor(
                result, primitive="S_SPIN", end_headings=reverse_required)
            self.assertIsNotNone(alignment)
            after = self.gen.generate(alignment.next_state, goal)
            reverse = find_successor(
                after, primitive="A_STRAIGHT", label="A_straight_goal_gear---")
            self.assertIsNotNone(reverse)
            self.assertAlmostEqual(reverse.edge.end_qf.x, 1.0, places=12)
            self.assertEqual(dict(reverse.edge.directions), directions(-1))
            self.assertTrue(reverse.edge.allow_reverse)

            forward_audit = audit_edges([forward.edge])
            reverse_audit = audit_edges([alignment.edge, reverse.edge])
            self.assertEqual(forward_audit["status"], FEASIBLE)
            self.assertEqual(reverse_audit["status"], FEASIBLE)

            passed(record, "forward_and_reverse_reach_same_goal",
                   forward=successor_summary(forward),
                   reverse=successor_summary(reverse),
                   forward_audit=forward_audit,
                   reverse_audit=reverse_audit)

    # -- S04 ---------------------------------------------------------------
    def test_S04_nonzero_pivot_two_directions_and_step_limit(self):
        """Both rotation directions at a nonzero pivot, and the 10 deg limit."""
        with record_case(
                "S04", "nonzero pivot two directions and fixed step audit",
                {"pivot": [0.0, 2.0], "cropped_deg": 90,
                 "fixed_step_deg": 10}) as record:
            pivot = (0.0, 2.0)
            dirs = directions()
            audits = {}
            for deg, sign in ((90, "pos"), (-90, "neg")):
                goal = FormationPose(0.0, 0.0, math.radians(deg))
                state = initial_state(
                    self.mdl, self.start_qf, self.start_headings,
                    config=self.cfg)
                result = self.gen.generate(state, goal)
                required = a_arc_required_headings(
                    self.mdl, self.start_qf, math.radians(deg), pivot, dirs,
                    limits=self.motion_limits)
                alignment = find_successor(
                    result, primitive="S_SPIN", end_headings=required)
                self.assertIsNotNone(alignment)
                after = self.gen.generate(alignment.next_state, goal)
                label = "A_arc_crop_p0_2_{}_gear+++".format(sign)
                arc = find_successor(after, primitive="A_ARC", label=label)
                self.assertIsNotNone(arc)
                audit = audit_edges([alignment.edge, arc.edge])
                self.assertEqual(audit["status"], FEASIBLE)
                audits[sign] = {
                    "arc": successor_summary(arc),
                    "audit": audit,
                    "delta_psi": arc.edge.control.delta_psi,
                }

            # The fixed +/-10 deg library generates both signs, but under the
            # accepted default limits its S8 duration is jerk-limited and the
            # conservative FR02 bound cannot certify it.  Recorded exactly;
            # the exact model peaks are below the limits.
            fixed = {}
            for deg, sign in ((10, "pos"), (-10, "neg")):
                goal = FormationPose(0.0, 0.0, math.radians(deg))
                state = initial_state(
                    self.mdl, self.start_qf, self.start_headings,
                    config=self.cfg)
                result = self.gen.generate(state, goal)
                required = a_arc_required_headings(
                    self.mdl, self.start_qf, math.radians(deg), pivot, dirs,
                    limits=self.motion_limits)
                alignment = find_successor(
                    result, primitive="S_SPIN", end_headings=required)
                self.assertIsNotNone(alignment)
                after = self.gen.generate(alignment.next_state, goal)
                label = "A_arc_p0_2_{}_gear+++".format(sign)
                arc = find_successor(after, primitive="A_ARC", label=label)
                self.assertIsNotNone(arc)
                plan = UniformTimedPlan.from_edges(
                    (alignment.edge, arc.edge), BODY_SHAPES)
                audit = audit_edges([alignment.edge, arc.edge])
                self.assertEqual(audit["status"], REJECTED)
                self.assertEqual(audit["failure_code"],
                                 UNIFORM_DYNAMIC_UNCONFIRMED)
                fixed[label] = {
                    "arc": successor_summary(arc),
                    "audit": audit,
                    "exact_peaks": exact_peaks(plan),
                    "duration_s": plan.duration_s,
                }

            write_json_artifact("arc_step_audit.json", {
                "schema": "fr03.arc_step_audit.v1",
                "cropped_nonzero_pivot": audits,
                "fixed_step_10deg": fixed,
                "note": "cropped angles near the goal heading are the "
                        "certifiable nonzero-pivot rotation path; the fixed "
                        "10 deg step is generated in both directions but its "
                        "jerk-limited S8 duration is not certifiable by the "
                        "conservative FR02 bound (not a physical exceedance)",
            })
            passed(record, "both_cropped_directions_feasible",
                   cropped=audits,
                   fixed_step=fixed)

    # -- S05 ---------------------------------------------------------------
    def test_S05_mid_sweep_and_spin_obstacles_rejected(self):
        """Obstacles hit only inside a sweep reject the successor."""
        with record_case(
                "S05", "mid-motion and mid-spin obstacles",
                {"map_resolution_m": MAP_RES,
                 "map_origin": [MAP_ORIGIN, MAP_ORIGIN]}) as record:
            # O translation: obstacle only under the middle of vehicle 1's
            # centre path, clear at both endpoints.
            o_edge = o_translation_edge(
                self.mdl, self.start_qf, self.start_headings, (0.0, 0.8),
                directions(), headings(math.pi / 2.0),
                limits=self.motion_limits)
            o_points = [(1.0 + dx, 0.45 + dy)
                        for dx in (-0.02, 0.0, 0.02)
                        for dy in (-0.02, 0.0, 0.02)]
            o_grid = grid_with(o_points)
            o_checker = checker(o_grid)
            self.assertTrue(o_checker.static_check(
                o_edge.start_qf, self.start_headings).success)
            self.assertTrue(o_checker.static_check(
                o_edge.end_qf, dict(o_edge.end_headings)).success)
            o_report = o_checker.swept_check_analytic(
                [analytic_sweep_segment(o_edge, 10.0)], SweepPolicy(0.01))
            self.assertFalse(o_report.success)
            self.assertEqual(o_report.reason, "MAP_OBB_COLLISION")
            self.assertGreater(o_report.station, 0.0)
            self.assertLess(o_report.station, 1.0)
            o_generator = generator(o_grid, self.cfg)
            o_state = initial_state(
                self.mdl, self.start_qf, self.start_headings, config=self.cfg)
            o_result = o_generator.generate(
                o_state, FormationPose(0.0, 0.8, 0.0))
            self.assertIn("MAP_OBB_COLLISION",
                          rejection_reasons(o_result))

            # S spin: obstacle outside both endpoint rectangles but inside
            # the swept rotated body.
            spin_edge = s_spin_edge(
                self.mdl, self.start_qf, self.start_headings,
                headings(math.pi), limits=self.motion_limits)
            spin_points = [(1.0 + dx, 0.40 + dy)
                           for dx in (-0.02, 0.0, 0.02)
                           for dy in (-0.02, 0.0, 0.02)]
            spin_grid = grid_with(spin_points)
            spin_checker = checker(spin_grid)
            self.assertTrue(spin_checker.static_check(
                spin_edge.start_qf, self.start_headings).success)
            self.assertTrue(spin_checker.static_check(
                spin_edge.end_qf, dict(spin_edge.end_headings)).success)
            spin_report = spin_checker.swept_check_analytic(
                [analytic_sweep_segment(spin_edge, 10.0)], SweepPolicy(0.01))
            self.assertFalse(spin_report.success)
            self.assertEqual(spin_report.vehicle_id, 1)
            self.assertGreater(spin_report.station, 0.0)
            self.assertLess(spin_report.station, 1.0)

            passed(record, "O_mid_path_obstacle",
                   obstacle_cells=o_points,
                   start_clear=True, end_clear=True,
                   reason=o_report.reason,
                   vehicle_id=o_report.vehicle_id,
                   station=o_report.station,
                   generator_rejections=rejection_reasons(o_result))
            passed(record, "S_mid_spin_obstacle",
                   obstacle_cells=spin_points,
                   start_clear=True, end_clear=True,
                   reason=spin_report.reason,
                   vehicle_id=spin_report.vehicle_id,
                   station=spin_report.station)

    # -- S06 ---------------------------------------------------------------
    def test_S06_wall_edge_angle_availability(self):
        """A wall admits one pivot direction and rejects the other."""
        with record_case(
                "S06", "wall edge: usable vs unusable angle",
                {"wall_x_m": 2.3, "pivot": [0.0, 2.0]}) as record:
            wall_points = [(2.3, y) for y in np.arange(-6.0, 6.0, MAP_RES)]
            grid = grid_with(wall_points)
            dirs = directions()
            outcomes = {}
            for sign, delta in (("pos", math.pi / 2.0),
                                ("neg", -math.pi / 2.0)):
                required = a_arc_required_headings(
                    self.mdl, self.start_qf, delta, (0.0, 2.0), dirs,
                    limits=self.motion_limits)
                edge = a_arc_edge(
                    self.mdl, self.start_qf, required, delta, (0.0, 2.0),
                    dirs, limits=self.motion_limits)
                chk = checker(grid)
                self.assertTrue(chk.static_check(
                    edge.start_qf, required).success)
                report = chk.swept_check_analytic(
                    [analytic_sweep_segment(edge, 10.0)], SweepPolicy(0.01))
                outcomes[sign] = {
                    "delta_psi": delta,
                    "start_clear": True,
                    "sweep_success": report.success,
                    "reason": report.reason,
                    "vehicle_id": report.vehicle_id,
                    "station": report.station,
                }
            self.assertFalse(outcomes["pos"]["sweep_success"])
            self.assertTrue(outcomes["neg"]["sweep_success"])

            # The generator applies the same distinction after S alignment.
            req_pos = a_arc_required_headings(
                self.mdl, self.start_qf, math.pi / 2.0, (0.0, 2.0), dirs,
                limits=self.motion_limits)
            state_pos = state_from_edge(s_spin_edge(
                self.mdl, self.start_qf, self.start_headings,
                {v: wrap_angle(req_pos[v] - self.start_headings[v])
                 for v in VEHICLE_IDS},
                limits=self.motion_limits))
            gen = generator(grid, self.cfg)
            result = gen.generate(
                state_pos, FormationPose(0.0, 0.0, math.pi / 2.0))
            self.assertIsNone(find_successor(
                result, primitive="A_ARC",
                label="A_arc_crop_p0_2_pos_gear+++"))
            self.assertIn("MAP_OBB_COLLISION", rejection_reasons(result))

            passed(record, "wall_angle_difference", outcomes=outcomes)

    # -- S07 ---------------------------------------------------------------
    def test_S07_free_terminal_heading(self):
        """A placeholder goal theta must not exclude a reachable state."""
        with record_case(
                "S07", "free terminal body heading",
                {"goal": [0.6, -0.4, 0.25]}) as record:
            gen = generator(cfg=self.cfg)
            goal = FormationPose(0.6, -0.4, 0.25)
            state = UniformSearchState(
                qf=goal,
                headings=tuple((vehicle_id, 1.1)
                               for vehicle_id in VEHICLE_IDS),
                mode=MODE_A,
                wheel=initial_wheel_signature(self.mdl, self.motion_limits),
                g=0.0)
            self.assertTrue(gen.is_goal(state, goal))
            self.assertTrue(gen.is_goal(
                state, goal, goal_headings={v: 1.1 for v in VEHICLE_IDS}))

            strict = generator(
                cfg=config(goal_heading_free=False))
            self.assertFalse(strict.is_goal(
                state, goal,
                goal_headings={v: 0.0 for v in VEHICLE_IDS}))
            self.assertTrue(strict.is_goal(
                state, goal,
                goal_headings={v: 1.1 for v in VEHICLE_IDS}))

            passed(record, "free_heading_goal_test",
                   goal_heading_free=True,
                   placeholder_heading_excluded=False,
                   strict_heading_required_when_disabled=True)

    # -- S08 ---------------------------------------------------------------
    def test_S08_every_legal_successor_class_passes_fr02(self):
        """Each legal successor class is audited by FR02, with real setup cost."""
        with record_case(
                "S08", "successor classes to FR02 and seconds-dimension cost",
                {"goal": [1.0, 0.6, math.pi / 2.0]}) as record:
            goal = FormationPose(1.0, 0.6, math.pi / 2.0)
            state = initial_state(
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg)
            result = self.gen.generate(state, goal)

            # A straight is only legal after its tangent headings are reached
            # through S; the direct-A form is rejected, not silently rotated.
            straight_required = a_straight_required_headings(
                self.mdl, (goal.x - state.qf.x, goal.y - state.qf.y),
                directions())
            straight_alignment = find_successor(
                result, primitive="S_SPIN", end_headings=straight_required)
            self.assertIsNotNone(straight_alignment)
            straight_result = self.gen.generate(
                straight_alignment.next_state, goal)
            straight = find_successor(
                straight_result, primitive="A_STRAIGHT",
                label="A_straight_goal_gear+++")
            self.assertIsNotNone(straight)
            o_item = find_successor(
                result, primitive="O_TRANSLATION", label_prefix="O_goal")
            self.assertIsNotNone(o_item)
            o_branch = find_successor(result, primitive="O_TRANSLATION")
            self.assertIsNotNone(o_branch)

            pivot = (0.0, 2.0)
            required = a_arc_required_headings(
                self.mdl, self.start_qf, math.pi / 2.0, pivot,
                directions(), limits=self.motion_limits)
            alignment = find_successor(
                result, primitive="S_SPIN", end_headings=required)
            self.assertIsNotNone(alignment)
            after = self.gen.generate(alignment.next_state, goal)
            arc = find_successor(
                after, primitive="A_ARC",
                label="A_arc_crop_p0_2_pos_gear+++")
            self.assertIsNotNone(arc)

            classes = {
                "A_STRAIGHT": ([straight_alignment.edge, straight.edge],
                               [straight_alignment, straight]),
                "O_TRANSLATION": ([o_item.edge], [o_item]),
                "S_SPIN": ([alignment.edge], [alignment]),
                "A_ARC_CROPPED": ([alignment.edge, arc.edge],
                                  [alignment, arc]),
            }
            audits = {}
            for name, (edges, items) in classes.items():
                audit = audit_edges(edges)
                self.assertEqual(audit["status"], FEASIBLE, name)
                self.assertTrue(audit["feasible"], name)
                worst = 0.0
                for item in items:
                    error = continuity_error(item.next_state.parent, item)
                    worst = max(worst, error["position_m"],
                                error["psi_rad"], error["heading_rad"])
                self.assertLessEqual(worst, CONTINUITY_TOL, name)
                audits[name] = {
                    "audit": audit,
                    "continuity_sup": worst,
                    "successors": [successor_summary(item)
                                   for item in items],
                }

            # Cost equals the actual FR02 setup demand plus the proxy.  The S
            # edge changes mode from the initial A configuration, so FR02
            # emits an initial setup phase to compare against.
            single = UniformTimedPlan.from_edges(
                (alignment.edge,), BODY_SHAPES)
            setup_phases = [phase for phase in single.phases
                            if phase.kind == "initial_setup"]
            self.assertEqual(len(setup_phases), 1)
            recomputed = setup_demand_s(
                state.wheel, alignment.edge, self.motion_limits)
            self.assertAlmostEqual(
                recomputed, setup_phases[0].duration_s, places=9)
            self.assertAlmostEqual(
                alignment.setup_s, setup_phases[0].duration_s, places=9)
            self.assertAlmostEqual(
                alignment.cost_s,
                alignment.setup_s + alignment.motion_proxy_s, places=12)
            proxy = motion_proxy_s(alignment.edge, self.cfg.timing_limits)
            self.assertAlmostEqual(proxy, alignment.motion_proxy_s,
                                   places=12)
            actual_motion_s = single.duration_s - setup_phases[0].duration_s
            self.assertGreater(abs(proxy - actual_motion_s), 1e-6)
            # A no-wheel-change A straight starts with zero setup demand.
            direct = a_straight_edge(
                self.mdl, self.start_qf, self.start_headings, (1.0, 0.0),
                directions(), limits=self.motion_limits)
            self.assertEqual(
                setup_demand_s(state.wheel, direct, self.motion_limits), 0.0)
            # The straight after S keeps its required A wheel configuration,
            # so the S->A mode change is charged instead of being hidden.
            self.assertGreater(straight.setup_s, 0.0)

            write_json_artifact("successor_fr02_audit.json", {
                "schema": "fr03.successor_fr02_audit.v1",
                "goal": [goal.x, goal.y, goal.psi],
                "classes": audits,
                "cost": {
                    "definition": "actual FR02 common stationary setup "
                                  "demand + first-order center/yaw motion "
                                  "estimate (seconds); not the S8 T_end and "
                                  "not admissible",
                    "A_straight": {
                        "setup_s": straight.setup_s,
                        "motion_proxy_s": straight.motion_proxy_s,
                        "cost_s": straight.cost_s,
                        "fr02_initial_setup_s":
                            setup_phases[0].duration_s,
                        "fr02_motion_s": audits["A_STRAIGHT"]["audit"]
                            ["duration_s"] - setup_phases[0].duration_s,
                    },
                },
            })
            passed(record, "all_legal_classes_feasible", classes=audits)

    # -- S09 ---------------------------------------------------------------
    def test_S09_fixed_seed_200_state_measurement(self):
        """Fixed-seed local measurement over 200 exact states."""
        with record_case(
                "S09", "fixed-seed 200-state local successor measurement",
                {"seed": 20261114, "states": 200,
                 "goal": [5.0, 3.0, 0.4]}) as record:
            seed = 20261114
            rng = random.Random(seed)
            goal = FormationPose(5.0, 3.0, 0.4)
            gen = generator(cfg=self.cfg)
            state = initial_state(
                self.mdl, self.start_qf, self.start_headings, config=self.cfg)
            measurements = []
            walk_states = 0
            fallback_states = 0

            tracemalloc.start()
            for index in range(200):
                result = gen.generate(state, goal)
                self.assertEqual(
                    result.candidate_count,
                    len(result.successors) + len(result.rejections))
                measurements.append({
                    "index": index,
                    "origin": "walk",
                    "qf": [state.qf.x, state.qf.y, state.qf.psi],
                    "mode": state.mode,
                    "g": state.g,
                    "key": make_key(state, self.cfg).as_dict(),
                    "candidate_count": result.candidate_count,
                    "accepted_count": len(result.successors),
                    "rejection_histogram": result.rejection_histogram(),
                    "generation_s": result.generation_s,
                    "collision_s": result.collision_s,
                    "successor_labels": [item.label
                                         for item in result.successors],
                })
                if result.successors:
                    state = rng.choice(result.successors).next_state
                    walk_states += 1
                else:
                    fallback_states += 1
                    state = UniformSearchState(
                        qf=FormationPose(
                            rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0),
                            rng.uniform(-math.pi, math.pi)),
                        headings=tuple(
                            (vehicle_id, rng.uniform(-math.pi, math.pi))
                            for vehicle_id in VEHICLE_IDS),
                        mode=MODE_A,
                        wheel=initial_wheel_signature(
                            self.mdl, self.motion_limits),
                        g=0.0)
                    measurements[-1]["origin"] = "fallback"
            current_bytes, peak_bytes = tracemalloc.get_traced_memory()
            tracemalloc.stop()

            self.assertEqual(len(measurements), 200)
            self.assertGreater(walk_states, 0)
            candidates = [m["candidate_count"] for m in measurements]
            generation = [m["generation_s"] for m in measurements]
            collision = [m["collision_s"] for m in measurements]
            histogram = {}
            for item in measurements:
                for reason, count in item["rejection_histogram"].items():
                    histogram[reason] = histogram.get(reason, 0) + count
            payload = {
                "schema": "fr03.measure_200_states.v1",
                "seed": seed,
                "state_count": len(measurements),
                "walk_states": walk_states,
                "fallback_states": fallback_states,
                "goal": [goal.x, goal.y, goal.psi],
                "fixed_bins": {
                    "xy_bin_m": self.cfg.xy_bin_m,
                    "psi_bin_rad": self.cfg.psi_bin_rad,
                    "heading_bin_rad": self.cfg.heading_bin_rad,
                },
                "candidate_count": {
                    "total": sum(candidates),
                    "mean": statistics.mean(candidates),
                    "median": statistics.median(candidates),
                    "minimum": min(candidates),
                    "maximum": max(candidates),
                },
                "rejection_histogram": histogram,
                "generation_s": {
                    "total": sum(generation),
                    "mean": statistics.mean(generation),
                    "maximum": max(generation),
                },
                "collision_s": {
                    "total": sum(collision),
                    "mean": statistics.mean(collision),
                    "maximum": max(collision),
                },
                "peak_memory_bytes": peak_bytes,
                "current_memory_bytes": current_bytes,
                "measurements": measurements,
                "note": "a local per-state successor measurement on an empty "
                        "map, not a planning-success or global-performance "
                        "claim; no full S8/S9 retiming runs per expansion",
            }
            write_json_artifact("measure_200_states.json", payload)

            self.assertTrue(all(m > 0 for m in candidates))
            self.assertTrue(all(value >= 0.0 for value in generation))
            self.assertTrue(all(value >= 0.0 for value in collision))
            passed(record, "200_states_measured",
                   state_count=len(measurements),
                   walk_states=walk_states,
                   fallback_states=fallback_states,
                   candidate_total=payload["candidate_count"]["total"],
                   candidate_mean=payload["candidate_count"]["mean"],
                   rejection_histogram=histogram,
                   generation_total_s=payload["generation_s"]["total"],
                   collision_total_s=payload["collision_s"]["total"],
                   mean_generation_s=payload["generation_s"]["mean"],
                   peak_memory_bytes=peak_bytes,
                   peak_memory_mb=peak_bytes / (1024.0 * 1024.0))

    # -- S10 ---------------------------------------------------------------
    def test_S10_expansion_does_not_run_full_timing_or_audit(self):
        """Successor expansion calls neither S8 scheduling nor the S9 audit."""
        with record_case(
                "S10", "no full S8/S9 per expansion",
                {"patched": ["uniform_timing.schedule_uniform_edges",
                             "uniform_quality.audit_uniform_timed_plan"]}) as record:
            import fwis_v3_optimizer.uniform_quality as quality_module
            import fwis_v3_optimizer.uniform_timing as timing_module

            calls = {"timing": 0, "audit": 0}

            def forbidden_timing(*args, **kwargs):
                calls["timing"] += 1
                raise AssertionError("S8 scheduler called during expansion")

            def forbidden_audit(*args, **kwargs):
                calls["audit"] += 1
                raise AssertionError("S9 audit called during expansion")

            original_timing = timing_module.schedule_uniform_edges
            original_audit = quality_module.audit_uniform_timed_plan
            timing_module.schedule_uniform_edges = forbidden_timing
            quality_module.audit_uniform_timed_plan = forbidden_audit
            try:
                gen = generator(cfg=self.cfg)
                state = initial_state(
                    self.mdl, self.start_qf, self.start_headings,
                    config=self.cfg)
                result = gen.generate(state, FormationPose(2.0, 1.0, 0.5))
            finally:
                timing_module.schedule_uniform_edges = original_timing
                quality_module.audit_uniform_timed_plan = original_audit

            self.assertEqual(calls, {"timing": 0, "audit": 0})
            self.assertGreater(result.candidate_count, 0)
            self.assertGreater(len(result.successors), 0)
            passed(record, "expansion_is_local_only",
                   timing_calls=calls["timing"],
                   audit_calls=calls["audit"],
                   accepted=len(result.successors),
                   candidates=result.candidate_count)

    # -- results -----------------------------------------------------------
    @classmethod
    def tearDownClass(cls):
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        source_hashes = {}
        import_paths = {}
        for name in SOURCE_FILES:
            path = PACKAGE_SRC / "fwis_v3_optimizer" / name
            source_hashes[name] = hashlib.sha256(
                path.read_bytes()).hexdigest()
            module = sys.modules.get(
                "fwis_v3_optimizer." + name[:-3])
            import_paths[name[:-3]] = getattr(module, "__file__", None)
        payload = {
            "schema": "fr03.successors.v1",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "status": "PASSED" if RESULTS and all(
                item["status"] == "PASSED" for item in RESULTS) else "FAILED",
            "python": sys.version,
            "platform": platform.platform(),
            "repo_root": str(REPO_ROOT),
            "package_src": str(PACKAGE_SRC),
            "import_paths": import_paths,
            "source_hashes": source_hashes,
            "contract": {
                "state": "exact q_F, exact unwrapped body headings, mode, "
                         "previous canonical wheel/primitive signature",
                "key": "finite labels only: XY 0.20 m, psi 10 deg, body "
                       "heading 10 deg plus gear, mode and wheel control; a "
                       "dedup label, not a dominance certificate",
                "successors": "fixed A pivot library c=(0,0),(0,+/-2),"
                              "(+/-2,0) with +/-10 deg steps plus the "
                              "goal-cropped angle, O eight cargo directions "
                              "plus goal, S alignment to required A headings",
                "legality": "FR01 authoritative edge derivation + real "
                            "analytic-pose OBB sweep; full S8/S9 is not run "
                            "per expansion",
                "cost": "actual FR02 common setup demand + first-order "
                        "center/yaw motion proxy in seconds; not the S8 "
                        "T_end, not admissible, not optimal",
            },
            "cases": RESULTS,
        }
        RESULTS_PATH.write_text(json.dumps(
            payload, indent=2, sort_keys=True, allow_nan=False) + "\n")


if __name__ == "__main__":
    unittest.main(verbosity=2)