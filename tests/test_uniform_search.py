"""FR03 S01--S12: finite-label states, keys, successors and proxy cost.

Scope is exactly the FR03 contract: exact states with ``q_F``, exact body
headings, mode and previous wheel/primitive signature; a finite deduplication
key (0.20 m XY, 10 deg psi/heading, mode, canonical wheel labels and the
finite A-scheme capability labels); O/A/S successors from the fixed library;
the real analytic-pose OBB sweep; and a seconds-dimension proxy cost made of
the actual FR02 setup demand plus a first-order motion estimate.  No
whole-map search, ROS or NLP is implemented.

S11/S12 are the FR03-R1 rework: a finite ``a_labels`` set records exactly
which A schemes the exact headings unlock under the current goal, so a legal
small-angle S alignment is not swallowed by a same-bin lower-g parent, while
pure duplicates and worse-g states are still deduplicated.

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
    a_capability_labels,
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


def state_from_edge(edge, goal_qf=None, cfg=None):
    cfg = cfg or config()
    return UniformSearchState(
        qf=edge.end_qf, headings=tuple(edge.end_headings), mode=edge.mode,
        wheel=wheel_signature_from_edge(edge), g=0.0,
        a_labels=a_capability_labels(
            model(), edge.end_qf, edge.end_headings, goal_qf, cfg,
            timing_motion_limits(cfg.timing_limits)))


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
                config=self.cfg, goal_qf=goal)
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
                config=self.cfg, goal_qf=goal)
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
                    config=self.cfg, goal_qf=goal)
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
                    config=self.cfg, goal_qf=goal)
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
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg,
                goal_qf=FormationPose(0.0, 0.8, 0.0))
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
            state_pos = state_from_edge(
                s_spin_edge(
                    self.mdl, self.start_qf, self.start_headings,
                    {v: wrap_angle(req_pos[v] - self.start_headings[v])
                     for v in VEHICLE_IDS},
                    limits=self.motion_limits),
                goal_qf=FormationPose(0.0, 0.0, math.pi / 2.0),
                cfg=self.cfg)
            # The exact headings satisfy the cropped-arc required set, so the
            # finite capability label is present before generation.
            self.assertIn("A_arc_crop_p0_2_pos_gear+++",
                          state_pos.a_labels)
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
                config=self.cfg, goal_qf=goal)
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
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg, goal_qf=goal)
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
                goal = FormationPose(2.0, 1.0, 0.5)
                state = initial_state(
                    self.mdl, self.start_qf, self.start_headings,
                    config=self.cfg, goal_qf=goal)
                result = gen.generate(state, goal)
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

    # -- S11 (FR03-R1) ----------------------------------------------------
    def test_S11_small_angle_S_alignment_kept_by_finite_labels(self):
        """A same-bin S alignment that unlocks A must survive the table."""
        with record_case(
                "S11", "FR03-R1: finite labels keep a small-angle S alignment",
                {"start": [0.0, 0.0, 0.0], "goal": [1.0, 0.0, 0.0],
                 "prefix_deg": [0.02 * 180.0 / math.pi,
                                -0.02 * 180.0 / math.pi],
                 "alignment_deg": [0.01 * 180.0 / math.pi,
                                   -0.01 * 180.0 / math.pi]}) as record:
            qf = FormationPose(0.0, 0.0, 0.0)
            goal = FormationPose(1.0, 0.0, 0.0)
            outcomes = {}
            for sign, name, next_direction in (
                    (1, "plus_sign", -1), (-1, "mirror_sign", 1)):
                # Legal prefix S: 0.02 -> 0.01 rad (mirror: -0.02 -> -0.01).
                prefix = s_spin_edge(
                    self.mdl, qf, headings(sign * 0.02),
                    headings(-sign * 0.01), limits=self.motion_limits)
                parent = state_from_edge(prefix, goal_qf=goal, cfg=self.cfg)
                # The prefix is an all-S state: no A capability is recorded.
                self.assertEqual(parent.a_labels, ())
                self.assertEqual(
                    dict(prefix.directions)[1], next_direction)

                # Exact 0.01 rad headings cannot emit the goal straight and
                # the generator instead offers the S alignment to 0 rad.
                before = self.gen.generate(parent, goal)
                self.assertFalse(any(
                    item.edge.primitive == "A_STRAIGHT"
                    for item in before.successors))
                self.assertIn("A_HEADING_REQUIRES_S_ALIGNMENT",
                              rejection_reasons(before))
                alignment = find_successor(
                    before, primitive="S_SPIN",
                    end_headings=headings(0.0))
                self.assertIsNotNone(alignment)
                self.assertGreater(alignment.cost_s, 0.0)
                child = alignment.next_state
                self.assertEqual(child.mode, MODE_S)
                self.assertEqual(
                    dict(alignment.edge.directions)[1], next_direction)

                # The child unlocks the goal-directed straight, so its finite
                # capability label must differ from the parent's.
                self.assertIn("A_straight_goal_gear+++", child.a_labels)
                parent_key = make_key(parent, self.cfg)
                child_key = make_key(child, self.cfg)
                self.assertEqual(parent_key.xy_bin, child_key.xy_bin)
                self.assertEqual(parent_key.psi_bin, child_key.psi_bin)
                self.assertEqual(parent_key.heading_label,
                                 child_key.heading_label)
                self.assertEqual(parent_key.a_labels, ())
                self.assertEqual(tuple(child_key.a_labels),
                                 tuple(child.a_labels))
                self.assertNotEqual(parent_key, child_key)

                # Offer the parent first (lower g), then the real generator
                # child: the child must be retained, not deduplicated.
                table = RepresentativeTable(self.cfg)
                accepted_parent, _, incumbent = table.offer(parent)
                self.assertTrue(accepted_parent)
                self.assertIsNone(incumbent)
                accepted_child, offered_key, incumbent = table.offer(child)
                self.assertTrue(
                    accepted_child,
                    "small-angle S alignment child was deduplicated by its "
                    "same-bin parent")
                self.assertIsNone(incumbent)
                self.assertEqual(offered_key, child_key)
                self.assertEqual(len(table), 2)
                self.assertLess(parent.g, child.g)
                self.assertEqual(
                    table.representative(child_key).qf.x, child.qf.x)

                # A pure duplicate and a worse-g state are still deduplicated
                # under the same key, including the same finite label set.
                duplicate = UniformSearchState(
                    qf=child.qf, headings=child.headings, mode=child.mode,
                    wheel=child.wheel, g=child.g, a_labels=child.a_labels)
                accepted_duplicate, _, duplicate_incumbent = table.offer(
                    duplicate)
                self.assertFalse(accepted_duplicate)
                self.assertIs(duplicate_incumbent, child)
                worse = UniformSearchState(
                    qf=child.qf, headings=child.headings, mode=child.mode,
                    wheel=child.wheel, g=child.g + 5.0,
                    a_labels=child.a_labels)
                accepted_worse, _, worse_incumbent = table.offer(worse)
                self.assertFalse(accepted_worse)
                self.assertIs(worse_incumbent, child)
                self.assertEqual(len(table), 2)

                # The retained child still produces the needed A straight.
                after = self.gen.generate(child, goal)
                straight = find_successor(
                    after, primitive="A_STRAIGHT",
                    label="A_straight_goal_gear+++")
                self.assertIsNotNone(straight)
                straight_continuity = continuity_error(child, straight)
                self.assertLessEqual(max(
                    straight_continuity["position_m"],
                    straight_continuity["psi_rad"],
                    straight_continuity["heading_rad"]), CONTINUITY_TOL)
                alignment_continuity = continuity_error(parent, alignment)
                self.assertLessEqual(max(
                    alignment_continuity["position_m"],
                    alignment_continuity["psi_rad"],
                    alignment_continuity["heading_rad"]), CONTINUITY_TOL)

                audit = audit_edges(
                    [prefix, alignment.edge, straight.edge])
                self.assertEqual(audit["status"], FEASIBLE)
                self.assertTrue(audit["feasible"])

                outcomes[name] = {
                    "prefix": {
                        "label": "S_prefix_{}".format(name),
                        "primitive": prefix.primitive,
                        "mode": prefix.mode,
                        "delta_psi": prefix.control.delta_psi,
                        "directions": {
                            str(vehicle_id): int(value)
                            for vehicle_id, value in prefix.directions},
                        "start_qf": [prefix.start_qf.x, prefix.start_qf.y,
                                     prefix.start_qf.psi],
                        "end_qf": [prefix.end_qf.x, prefix.end_qf.y,
                                   prefix.end_qf.psi],
                        "start_headings": {
                            str(vehicle_id): theta
                            for vehicle_id, theta in prefix.start_headings},
                        "end_headings": {
                            str(vehicle_id): theta
                            for vehicle_id, theta in prefix.end_headings},
                        "setup_s": setup_demand_s(
                            initial_wheel_signature(
                                self.mdl, self.motion_limits),
                            prefix, self.motion_limits),
                        "motion_proxy_s": motion_proxy_s(
                            prefix, self.cfg.timing_limits),
                    },
                    "parent_a_labels": list(parent.a_labels),
                    "child_a_labels": list(child.a_labels),
                    "parent_state": parent.as_dict(),
                    "child_state": child.as_dict(),
                    "parent_key": parent_key.as_dict(),
                    "child_key": child_key.as_dict(),
                    "kept_representative_for_child_key":
                        table.representative(child_key).as_dict(),
                    "identical_bins": {
                        "xy": parent_key.xy_bin == child_key.xy_bin,
                        "psi": parent_key.psi_bin == child_key.psi_bin,
                        "heading": parent_key.heading_label
                        == child_key.heading_label},
                    "table": {
                        "accepted_parent": accepted_parent,
                        "accepted_child": accepted_child,
                        "rejected_duplicate": not accepted_duplicate,
                        "rejected_worse_g": not accepted_worse,
                        "representatives": len(table),
                    },
                    "alignment": successor_summary(alignment),
                    "straight": successor_summary(straight),
                    "audit": audit,
                }

            write_json_artifact("rework_r1_small_angle.json", {
                "schema": "fr03.rework_r1_small_angle.v1",
                "goal": [goal.x, goal.y, goal.psi],
                "cases": outcomes,
                "note": "the prefix S leaves the all-S state at 0.01 rad "
                        "with no A capability label; the real generator's S "
                        "alignment child reaches 0 rad and records the "
                        "goal-directed straight label, so the two states are "
                        "in the same coarse bins but have different keys and "
                        "the lower-g parent can no longer swallow the child",
            })
            passed(record, "both_signs_child_retained_and_A_generated",
                   cases=outcomes)

    # -- S12 (FR03-R1 labels) ---------------------------------------------
    def test_S12_finite_label_domain_and_consistency(self):
        """Labels are finite and equal the A schemes the generator accepts."""
        with record_case(
                "S12", "FR03-R1: finite label domain and consistency",
                {"goals": {"straight": [1.0, 0.0, 0.0],
                           "reverse": [1.0, 0.6, 0.0],
                           "arc": [0.0, 0.0, math.pi / 2.0],
                           "nonzero_psi": [1.3, 0.4, 0.4]},
                 "note": "labels compared with the real generator on an "
                         "empty map; no pose or heading is snapped"}) as record:
            straight_goal = FormationPose(1.0, 0.0, 0.0)
            goal = FormationPose(1.0, 0.6, 0.0)
            cases = []

            # (a) initial A state at the origin with its tangent equal to the
            # goal direction: the goal straight is unlocked immediately.
            cases.append(("initial_goal_straight", initial_state(
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg, goal_qf=straight_goal), straight_goal))

            # (b) reverse-aligned state: S to the anti-tangent headings.
            direct = self.gen.generate(initial_state(
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg, goal_qf=goal), goal)
            reverse_required = a_straight_required_headings(
                self.mdl, (goal.x, goal.y), directions(-1))
            reverse_alignment = find_successor(
                direct, primitive="S_SPIN", end_headings=reverse_required)
            self.assertIsNotNone(reverse_alignment)
            cases.append(("reverse_aligned", reverse_alignment.next_state,
                          goal))

            # (c) nonzero pivot cropped arc: S to its required headings.
            pivot = (0.0, 2.0)
            arc_goal = FormationPose(0.0, 0.0, math.pi / 2.0)
            arc_required = a_arc_required_headings(
                self.mdl, self.start_qf, math.pi / 2.0, pivot, directions(),
                limits=self.motion_limits)
            arc_alignment = find_successor(
                self.gen.generate(initial_state(
                    self.mdl, self.start_qf, self.start_headings,
                    config=self.cfg, goal_qf=arc_goal), arc_goal),
                primitive="S_SPIN", end_headings=arc_required)
            self.assertIsNotNone(arc_alignment)
            cases.append(("nonzero_pivot_crop", arc_alignment.next_state,
                          arc_goal))

            # (d) nonzero cargo psi: align to the straight required headings
            # from a yawed start pose.
            yawed = FormationPose(0.3, -0.2, 0.4)
            yawed_goal = FormationPose(1.3, 0.4, 0.4)
            yawed_required = a_straight_required_headings(
                self.mdl, (yawed_goal.x - yawed.x, yawed_goal.y - yawed.y),
                directions())
            yawed_alignment = find_successor(
                self.gen.generate(initial_state(
                    self.mdl, yawed, headings(0.0), config=self.cfg,
                    goal_qf=yawed_goal), yawed_goal),
                primitive="S_SPIN", end_headings=yawed_required)
            self.assertIsNotNone(yawed_alignment)
            cases.append(("nonzero_psi", yawed_alignment.next_state,
                          yawed_goal))

            expected_qf = {
                "initial_goal_straight": (0.0, 0.0, 0.0),
                "reverse_aligned": (0.0, 0.0, 0.0),
                "nonzero_pivot_crop": (0.0, 0.0, 0.0),
                "nonzero_psi": (0.3, -0.2, 0.4),
            }
            results = {}
            for name, state, case_goal in cases:
                labels = tuple(state.a_labels)
                self.assertEqual(labels, tuple(sorted(set(labels))))
                # Labels are finite and drawn only from the bounded library.
                self.assertLessEqual(len(labels), 8 + 20 * 8)

                result = self.gen.generate(state, case_goal)
                accepted = sorted(
                    item.label for item in result.successors
                    if item.edge.primitive in ("A_STRAIGHT", "A_ARC"))
                self.assertEqual(sorted(labels), accepted, name)
                # A positive direction/curve label is present exactly when
                # the generator emits that A scheme; no label alone implies a
                # dynamic audit result (that stays a final-stage decision).
                self.assertGreaterEqual(len(accepted), 1, name)

                # No pose or heading snapping: the exact state is untouched.
                self.assertEqual(
                    [state.qf.x, state.qf.y, state.qf.psi],
                    list(expected_qf[name]))
                key = make_key(state, self.cfg)
                self.assertIsInstance(key.xy_bin[0], int)
                self.assertEqual(tuple(key.a_labels), labels)
                if name == "nonzero_psi":
                    # 0.5404 rad is not a 10 deg bin centre; the exact value
                    # survives in the state and in the edge start pose.
                    tangent = math.atan2(0.6, 1.0)
                    for _, theta in state.headings:
                        self.assertLess(
                            abs(wrap_angle(theta - tangent)), 1e-12)
                    self.assertGreater(
                        abs(tangent / self.cfg.heading_bin_rad
                            - round(tangent / self.cfg.heading_bin_rad)),
                        0.05)

                # Mode and wheel differences still produce different keys
                # even when the finite A labels are identical.
                wheel_mode_changed = UniformSearchState(
                    qf=state.qf, headings=state.headings,
                    mode=(MODE_O if state.mode != MODE_O else MODE_A),
                    wheel=tuple(
                        type(item)(
                            vehicle_id=item.vehicle_id,
                            mode=(MODE_O if item.mode != MODE_O else MODE_A),
                            direction=item.direction, control=item.control,
                            angles=item.angles)
                        for item in state.wheel),
                    g=state.g, a_labels=state.a_labels)
                wheel_control_changed = UniformSearchState(
                    qf=state.qf, headings=state.headings, mode=state.mode,
                    wheel=tuple(
                        type(item)(
                            vehicle_id=item.vehicle_id, mode=item.mode,
                            direction=item.direction,
                            control=item.control + 0.02,
                            angles=item.angles)
                        for item in state.wheel),
                    g=state.g, a_labels=state.a_labels)
                self.assertNotEqual(make_key(state, self.cfg),
                                    make_key(wheel_mode_changed, self.cfg))
                self.assertNotEqual(make_key(state, self.cfg),
                                    make_key(wheel_control_changed, self.cfg))

                results[name] = {
                    "a_labels": list(labels),
                    "accepted_A_labels": accepted,
                    "state": state.as_dict(),
                    "key": key.as_dict(),
                    "mode_and_wheel_still_distinguish_keys": True,
                    "successors": [
                        successor_summary(item) for item in result.successors
                        if item.edge.primitive in ("A_STRAIGHT", "A_ARC")],
                }

            # Goal-absent input states have a clear rule: straight and
            # cropped-arc labels cannot be evaluated, so they are omitted
            # (this only splits keys, never merges different capabilities).
            goal_free = initial_state(
                self.mdl, self.start_qf, self.start_headings,
                config=self.cfg)
            self.assertEqual(goal_free.a_labels, ())

            write_json_artifact("rework_r1_label_domain.json", {
                "schema": "fr03.rework_r1_label_domain.v1",
                "label_rule": "a label is present iff the exact headings "
                              "satisfy the FR01 required-heading set of that "
                              "configuration scheme under the same goal, "
                              "allow_reverse and gear combinations used by "
                              "the generator; fixed/cropped arc labels use "
                              "the same finite scheme list",
                "domain_bound": "8 goal-directed straight gear combos + "
                                "<=20 fixed/cropped arc schemes x 8 gear "
                                "combos",
                "goal_absent_rule": "straight and cropped-arc labels omitted",
                "cases": results,
            })
            passed(record, "labels_equal_generator_schemes",
                   cases={name: value["a_labels"]
                          for name, value in results.items()},
                   goal_absent_labels=list(goal_free.a_labels))

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
                       "heading 10 deg plus gear, mode wheel control and the "
                       "finite A-scheme capability label set; a dedup label, "
                       "not a dominance certificate",
                "a_labels": "finite A-scheme capability labels: present iff "
                            "the exact headings satisfy that scheme's FR01 "
                            "required-heading set under the search goal; the "
                            "domain is the goal straight plus the fixed/"
                            "cropped arc library x allowed gear combos; "
                            "goal-dependent, so one table is used per goal",
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