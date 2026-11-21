"""FR02-B B01--B09: independent V0.2 uniform-plan audit and tamper rejection.

The audit under test re-derives every edge field, the common-time schedule,
the analytic state at phase boundaries and interior times, the dynamic peak
candidates and conservative bounds, the projection contract, and static plus
analytic swept collisions.  These acceptance tests prove both directions:

* every legal A/O/S manual plan, zero-angle-S vehicle, reverse edge and hold
  passes the full S9/collision audit before and after a model roundtrip, and
  every complete FEASIBLE result carries a completed full-interval dynamic
  bound (candidate peak plus a same-phase derivative Lipschitz error);
* a coherently tampered plan/projection is rejected with a structured first
  failure, even when the tampering preserves the formation centre path or the
  projection's internal rigidity;
* an interval whose candidate peak is below the limit but whose conservative
  full-interval bound is not returns UNIFORM_DYNAMIC_UNCONFIRMED, is not
  feasible/executable, and cannot pass the publication gate.

Evidence is written to ``FR02/B/uniform_quality_results.json`` (override with
``FR02_QUALITY_RESULTS_PATH``) and ``FR02/B/collision_map_b06.json`` (override
the directory with ``FR02_ARTIFACT_DIR``).

Run from the V0.2 worktree root::

    FR02_QUALITY_RESULTS_PATH=evaluation/formation_refactor_v02/FR02/B/uniform_quality_results.json \
    FR02_ARTIFACT_DIR=evaluation/formation_refactor_v02/FR02/B \
    OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests \
        -p 'test_uniform_quality.py' -v
"""
from contextlib import contextmanager
from datetime import datetime, timezone
import copy
import dataclasses
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys
import unittest

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO_ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from fwis_v3_optimizer.core import Action, Grid, Pose
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import (
    FormationCollisionChecker,
    VehicleShape,
)
from fwis_v3_optimizer.formation_kinematics import ACKERMANN, SPIN
from fwis_v3_optimizer.formation_quality import (
    FEASIBLE,
    FORMATION_RETIMING_INFEASIBLE,
    FORMATION_SWEEP_COLLISION,
    INVALID,
    INVALID_REQUEST,
    REJECTED,
    FormationPublicationGate,
    FormationQualityConfig,
    _TAUS,
    grid_signature,
)
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.retiming import wheel_configuration
from fwis_v3_optimizer.uniform_motion import (
    a_arc_edge,
    a_arc_required_headings,
    a_straight_edge,
    o_translation_edge,
    s_spin_edge,
)
from fwis_v3_optimizer.uniform_quality import (
    UNIFORM_DYNAMIC_UNCONFIRMED,
    UNIFORM_MODEL_MISMATCH,
    UNIFORM_PHASE_SCHEDULE_INVALID,
    UNIFORM_PROJECTION_MISMATCH,
    UNIFORM_SWEEP_BUDGET_EXCEEDED,
    UniformAuditRequest,
    audit_uniform_timed_plan,
)
from fwis_v3_optimizer.uniform_timing import (
    MotionTiming,
    SetupTiming,
    timing_motion_limits,
)
from fwis_v3_optimizer.uniform_trajectory import (
    MODEL_VERSION,
    PROJECTION_SEMANTICS,
    UniformSampleProjection,
    UniformTimedPlan,
)

OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
BODY_SHAPES = {vehicle_id: VehicleShape(0.738, 0.500)
               for vehicle_id, _, _ in OFFSETS}
EMPTY_GRID = Grid(np.zeros((400, 400), dtype=bool), 0.05, -10.0, -10.0)
RESULTS_PATH = Path(os.environ.get(
    "FR02_QUALITY_RESULTS_PATH",
    str(REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR02" / "B"
        / "uniform_quality_results.json")))
ARTIFACT_DIR = Path(os.environ.get(
    "FR02_ARTIFACT_DIR",
    str(REPO_ROOT / "evaluation" / "formation_refactor_v02" / "FR02" / "B")))
RESULTS = []

SOURCE_FILES = (
    "uniform_quality.py",
    "uniform_trajectory.py",
    "formation_quality.py",
    "formation_collision.py",
    "formation_retiming.py",
)
ACCEPTANCE_THRESHOLDS = {
    "geometry_m_and_rad": 1e-7,
    "heading_rad": 1e-7,
    "rigidity_m": 1e-6,
    "mode_residual": 1e-6,
    "finite_difference": 1e-4,
    "time_continuity_s": 1e-9,
    "endpoint_velocity_acceleration": 1e-9,
    "dynamic_peak_ratio_task": 1e-3,
    "dynamic_peak_ratio_s9": 1e-6,
    "setup_pose_m_and_rad": 1e-9,
}


def model(ids_offsets=OFFSETS):
    return FormationModel(tuple(
        VehicleOffset(vehicle_id, x, y) for vehicle_id, x, y in ids_offsets))


def headings(value=0.0):
    return {vehicle_id: float(value) for vehicle_id, _, _ in OFFSETS}


def directions(value=1):
    return {vehicle_id: int(value) for vehicle_id, _, _ in OFFSETS}


def plan_for(edges, limits=None):
    return UniformTimedPlan.from_edges(
        tuple(edges), BODY_SHAPES, timing_limits=limits)


def straight_plan(distance=2.0):
    edge = a_straight_edge(
        model(), FormationPose(0.0, 0.0, 0.0), headings(), (distance, 0.0),
        directions())
    return plan_for((edge,))


def reverse_straight_plan():
    qf = FormationPose(0.0, 0.0, 0.0)
    alignment = s_spin_edge(
        model(), qf, headings(), {vehicle_id: math.pi
                                  for vehicle_id in headings()})
    edge = a_straight_edge(
        model(), qf, headings(math.pi), (2.0, 0.0), directions(-1),
        allow_reverse=True)
    return plan_for((alignment, edge))


def arc_plan():
    qf = FormationPose(2.0, -1.0, 0.35)
    start_headings = {1: 0.10, 2: -0.20, 3: 0.30}
    pivot = (0.0, 2.0)
    delta = 0.60
    arc_headings = a_arc_required_headings(
        model(), qf, delta, pivot, directions())
    edge = a_arc_edge(model(), qf, arc_headings, delta, pivot, directions())
    return plan_for((edge,))


def o_plan(reverse=False):
    angle = (-math.pi / 2.0) if reverse else (math.pi / 2.0)
    edge = o_translation_edge(
        model(), FormationPose(0.0, 0.0, 0.0), headings(), (0.0, 0.8),
        directions(-1 if reverse else 1), headings(angle),
        allow_reverse=reverse)
    return plan_for((edge,))


def spin_plan():
    edge = s_spin_edge(
        model(), FormationPose(0.0, 0.0, 0.0), headings(),
        {1: math.pi / 2.0, 2: 0.0, 3: -math.pi / 4.0})
    return plan_for((edge,))


def complex_plan():
    """Manual S alignment -> c=(0,2) A arc -> S -> O, with psi != 0."""
    qf = FormationPose(2.0, -1.0, 0.35)
    start_headings = {1: 0.10, 2: -0.20, 3: 0.30}
    delta_psi = 0.60
    pivot = (0.0, 2.0)
    arc_headings = a_arc_required_headings(
        model(), qf, delta_psi, pivot, directions())
    alignment = s_spin_edge(
        model(), qf, start_headings,
        {vehicle_id: arc_headings[vehicle_id] - start_headings[vehicle_id]
         for vehicle_id in start_headings})
    arc = a_arc_edge(
        model(), qf, dict(alignment.end_headings), delta_psi, pivot,
        directions())
    second_spin = s_spin_edge(
        model(), arc.end_qf, dict(arc.end_headings),
        {1: 0.13, 2: -0.17, 3: 0.11})
    translation_angle = math.atan2(0.70, 0.45)
    translation_headings = dict(second_spin.end_headings)
    translation = o_translation_edge(
        model(), arc.end_qf, translation_headings, (0.45, 0.70),
        directions(),
        {vehicle_id: translation_angle - translation_headings[vehicle_id]
         for vehicle_id in translation_headings})
    return plan_for((alignment, arc, second_spin, translation))


def o_same_mode_plan():
    """FR02-A04: O control change creates a setup, an identical O does not."""
    qf = FormationPose(0.0, 0.0, 0.0)
    first = o_translation_edge(
        model(), qf, headings(), (1.0, 0.0), directions(), headings(0.0))
    second = o_translation_edge(
        model(), first.end_qf, headings(), (0.0, 0.7), directions(),
        headings(math.pi / 2.0))
    third = o_translation_edge(
        model(), second.end_qf, headings(), (0.0, 0.4), directions(),
        headings(math.pi / 2.0))
    return plan_for((first, second, third))


def hold_plan():
    qf = FormationPose(1.0, -2.0, 0.4)
    hs = {1: 0.2, 2: -0.3, 3: 0.7}
    return UniformTimedPlan.from_edges(
        (), BODY_SHAPES, model=model(), start_qf=qf, start_headings=hs,
        expected_end_qf=qf, expected_end_headings=hs)


LEGAL_FIXTURES = (
    ("A_straight_forward", straight_plan),
    ("A_straight_reverse", reverse_straight_plan),
    ("A_arc_nonzero_pivot", arc_plan),
    ("O_lateral_forward", lambda: o_plan(False)),
    ("O_lateral_reverse", lambda: o_plan(True)),
    ("O_same_mode_wheel_change", o_same_mode_plan),
    ("S_spin_with_zero_angle_vehicle", spin_plan),
    ("S_A_S_O_complex", complex_plan),
    ("zero_motion_hold", hold_plan),
)


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


def _write_json_artifact(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def passed(record, subcase, **values):
    tamper = values.get("tamper")
    if tamper is not None:
        private = {key: tamper[key] for key in tamper
                   if key.startswith("_")}
        if private:
            evidence = {key: value for key, value in tamper.items()
                        if not key.startswith("_")}
            files = {}
            stem = "{}_{}".format(
                record["case"].lower(), subcase.replace("/", "_"))
            for key, payload in sorted(private.items()):
                name = "{}.{}.{}.json".format(
                    stem, evidence["kind"], key[1:])
                _write_json_artifact(ARTIFACT_DIR / "tampers" / name, payload)
                files[key[1:]] = "tampers/" + name
            evidence["input_files"] = files
            values["tamper"] = evidence
    record["subcases"].append(
        dict(name=subcase, status="PASSED", **values))


def write_model_artifact(name, plan):
    relative = "models/" + name + ".json"
    _write_json_artifact(
        ARTIFACT_DIR / relative, _input_payload("plan", plan))
    return relative


def _set(obj, name, value):
    """Bypass frozen-dataclass ``__post_init__`` for a deliberate tamper."""
    object.__setattr__(obj, name, value)


def _strip_factory(value):
    if isinstance(value, dict):
        return {key: _strip_factory(item) for key, item in value.items()
                if key != "factory"}
    if isinstance(value, (list, tuple)):
        return [_strip_factory(item) for item in value]
    return value


def _safe_value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if value is None or isinstance(value, (bool, int, str)):
        return value
    return repr(value)


def _json_ready(value):
    """Normalize nested payloads to JSON-safe string-keyed structures."""
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return _json_ready(dataclasses.asdict(value))
    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    return repr(value)


def _input_payload(kind, value):
    if kind == "plan":
        return _json_ready(_strip_factory(value.to_model_dict()))
    if kind == "projection":
        return _json_ready(value.as_dict())
    if kind == "config":
        return _json_ready(value)
    raise ValueError(kind)


def _payload_digest(payload):
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), allow_nan=True)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _payload_diff(before, after, path="", state=None):
    if state is None:
        state = {"changes": [], "count": 0, "truncated": False}
    if state["count"] >= 200:
        state["truncated"] = True
        return state
    if isinstance(before, dict) and isinstance(after, dict):
        before_keys = set(before)
        after_keys = set(after)
        if before_keys != after_keys:
            state["count"] += 1
            state["changes"].append({
                "path": path or "$",
                "before": sorted(before_keys),
                "after": sorted(after_keys),
            })
        for key in sorted(before_keys & after_keys):
            _payload_diff(before[key], after[key],
                          "{}.{}".format(path, key)
                          if path else str(key), state)
        return state
    if isinstance(before, (list, tuple)) and isinstance(
            after, (list, tuple)):
        if len(before) != len(after):
            state["count"] += 1
            state["changes"].append({
                "path": path or "$", "before_length": len(before),
                "after_length": len(after)})
        for index, (first, second) in enumerate(zip(before, after)):
            _payload_diff(first, second, "{}[{}]".format(path, index), state)
        return state
    if type(before) is not type(after) or before != after:
        state["count"] += 1
        state["changes"].append({
            "path": path or "$",
            "before": _safe_value(before),
            "after": _safe_value(after),
        })
    return state


def _live_plan_payload(plan):
    """Non-canonicalized edge fields, for tampers that reorder wheel IDs."""
    edges = []
    for edge in plan.edges:
        payload = dataclasses.asdict(edge)
        payload.pop("model", None)
        edges.append(payload)
    return _json_ready({"edges": edges})


def tamper_evidence(kind, before, after):
    payload_before = _input_payload(kind, before)
    payload_after = _input_payload(kind, after)
    diff = _payload_diff(payload_before, payload_after)
    evidence = {
        "kind": kind,
        "before_sha256": _payload_digest(payload_before),
        "after_sha256": _payload_digest(payload_after),
        "changed_count_at_least": int(diff["count"]),
        "changes_truncated": bool(diff["truncated"]),
        "changes": diff["changes"],
        "_before_payload": payload_before,
        "_after_payload": payload_after,
    }
    # ``to_model_dict`` canonicalizes wheel records by vehicle ID, so a
    # deliberate ID swap can round-trip to an identical model dict.  In that
    # case the live edge payload is the only faithful before/after record.
    if kind == "plan" and diff["count"] == 0:
        live_before = _live_plan_payload(before)
        live_after = _live_plan_payload(after)
        live_diff = _payload_diff(live_before, live_after)
        evidence["_live_before"] = live_before
        evidence["_live_after"] = live_after
        evidence["live_changed_count_at_least"] = int(live_diff["count"])
        evidence["live_changes_truncated"] = bool(live_diff["truncated"])
        evidence["live_changes"] = live_diff["changes"]
        evidence["note"] = (
            "canonical model dict is unchanged; live edge payloads record "
            "the field that was tampered")
    return evidence


def clone_plan(plan):
    return UniformTimedPlan.from_model_dict(plan.to_model_dict())


def clone_projection(projection):
    return copy.copy(projection)


def clone_timed(projection):
    timed = copy.copy(projection.timed_trajectory)
    _set(projection, "timed_trajectory", timed)
    return timed


def heading_pairs(mapping):
    return tuple((vehicle_id, float(mapping[vehicle_id]))
                 for vehicle_id in sorted(mapping))


def audit(plan, projection, grid=EMPTY_GRID, config=None):
    request = UniformAuditRequest.from_plan(plan)
    return audit_uniform_timed_plan(request, plan, projection, grid, config)


def failure_evidence(result):
    failure = result.failure
    return {
        "status": str(result.status),
        "reason": str(result.reason),
        "feasible": bool(result.feasible),
        "code": getattr(failure, "code", None),
        "field": getattr(failure, "field", None),
        "vehicle_id": getattr(failure, "vehicle_id", None),
        "other_vehicle_id": getattr(failure, "other_vehicle_id", None),
        "block_index": getattr(failure, "block_index", None),
        "segment_index": getattr(failure, "segment_index", None),
        "sample_index": getattr(failure, "sample_index", None),
        "time_s": getattr(failure, "time_s", None),
        "station": getattr(failure, "station", None),
        "constraint": getattr(failure, "constraint", None),
        "max_error": getattr(failure, "max_error", None),
        "detail": getattr(failure, "detail", None),
    }


def legal_metrics(result):
    metrics = result.metrics
    return {
        "status": str(result.status),
        "reason": str(result.reason),
        "duration_s": metrics.get("duration_s"),
        "model_version": metrics.get("model_version"),
        "model_sha256": metrics.get("model_sha256"),
        "projection_semantics": metrics.get("projection_semantics"),
        "independent_audit": metrics.get("independent_audit"),
        "collision_certificate": metrics.get("collision_certificate"),
        "map_version_checked": metrics.get("map_version_checked"),
        "map_signature": metrics.get("map_signature"),
        "static_collision_checks": metrics.get("static_collision_checks"),
        "max_state_error_m_or_rad": metrics.get(
            "max_state_error_m_or_rad"),
        "max_projection_error_m_or_rad": metrics.get(
            "max_projection_error_m_or_rad"),
        "max_rigidity_error_m": metrics.get("max_rigidity_error_m"),
        "max_finite_difference_error": metrics.get(
            "max_finite_difference_error"),
        "max_interior_geometry_error_m_or_rad": metrics.get(
            "max_interior_geometry_error_m_or_rad"),
        "max_wheel_start_error_rad": metrics.get(
            "max_wheel_start_error_rad"),
        "max_wheel_end_error_rad": metrics.get(
            "max_wheel_end_error_rad"),
        "max_normalized_mode_residual": metrics.get(
            "max_normalized_mode_residual"),
        "max_mode_residual_rad": metrics.get("max_mode_residual_rad"),
        "boundary_errors": metrics.get("boundary_errors"),
        "interior_counts": metrics.get("interior_counts"),
        "setup_evidence": metrics.get("setup_evidence"),
        "dynamic_peak_ratio": metrics.get("dynamic_peak_ratio"),
        "dynamic_bound_ratio": metrics.get("dynamic_bound_ratio"),
        "dynamic_confirmation": metrics.get("dynamic_confirmation"),
        "dynamic_continuous_certificate": metrics.get(
            "dynamic_continuous_certificate"),
        "dynamic_limitations": metrics.get("dynamic_limitations"),
        "dynamic_candidate_count": metrics.get("dynamic_candidate_count"),
        "dynamic_method": metrics.get("dynamic_method"),
        "dynamic_evidence": metrics.get("dynamic_evidence"),
        "sweep_policy": metrics.get("sweep_policy"),
        "sweep_coverage": metrics.get("sweep_coverage"),
        "first_collision": metrics.get("first_collision"),
        "minimum_obstacle_clearance_m": metrics.get(
            "minimum_obstacle_clearance_m"),
        "minimum_intervehicle_clearance_m": metrics.get(
            "minimum_intervehicle_clearance_m"),
    }


class UniformQualityAcceptance(unittest.TestCase):
    maxDiff = None

    # -- shared assertions ------------------------------------------------
    def assert_rejected(self, result, code=None, field_contains=None,
                        vehicle_id=None, constraint=None):
        self.assertNotEqual(result.status, FEASIBLE)
        self.assertFalse(result.feasible)
        self.assertIsNotNone(result.failure, "rejection must be structured")
        if code is not None:
            self.assertEqual(result.reason, code,
                             "expected {}, got {} ({})".format(
                                 code, result.reason, result.detail))
        if field_contains is not None:
            self.assertIn(field_contains, result.failure.field or "",
                          "field {!r} does not contain {!r}".format(
                              result.failure.field, field_contains))
        if vehicle_id is not None:
            self.assertEqual(result.failure.vehicle_id, vehicle_id)
        if constraint is not None:
            self.assertEqual(result.failure.constraint, constraint)

    def assert_legal(self, plan, result, clone_result=None):
        self.assertTrue(result.feasible, result.detail)
        self.assertEqual(result.status, FEASIBLE)
        self.assertEqual(result.reason, "OK")
        metrics = result.metrics
        self.assertTrue(metrics.get("independent_audit"))
        self.assertTrue(metrics.get("collision_certificate"))
        self.assertTrue(metrics.get("map_version_checked"))
        self.assertEqual(metrics.get("model_version"), MODEL_VERSION)
        self.assertEqual(metrics.get("projection_semantics"),
                         PROJECTION_SEMANTICS)
        self.assertLessEqual(metrics.get(
            "max_state_error_m_or_rad", 0.0),
            ACCEPTANCE_THRESHOLDS["geometry_m_and_rad"])
        self.assertLessEqual(metrics.get(
            "max_projection_error_m_or_rad", 0.0),
            ACCEPTANCE_THRESHOLDS["geometry_m_and_rad"])
        self.assertLessEqual(metrics.get(
            "max_rigidity_error_m", 0.0),
            ACCEPTANCE_THRESHOLDS["rigidity_m"])
        self.assertLessEqual(metrics.get(
            "max_finite_difference_error", 0.0),
            ACCEPTANCE_THRESHOLDS["finite_difference"])
        self.assertLessEqual(metrics.get(
            "max_interior_geometry_error_m_or_rad", 0.0),
            ACCEPTANCE_THRESHOLDS["geometry_m_and_rad"])
        self.assertLessEqual(metrics.get("max_wheel_start_error_rad", 0.0),
                             ACCEPTANCE_THRESHOLDS["geometry_m_and_rad"])
        self.assertLessEqual(metrics.get("max_wheel_end_error_rad", 0.0),
                             ACCEPTANCE_THRESHOLDS["geometry_m_and_rad"])
        self.assertLessEqual(metrics.get(
            "max_normalized_mode_residual", 0.0),
            ACCEPTANCE_THRESHOLDS["mode_residual"])
        self.assertLessEqual(metrics.get("max_mode_residual_rad", 0.0),
                             1e-7)
        for setup in metrics.get("setup_evidence", ()):
            self.assertGreaterEqual(
                setup["actual_duration_s"],
                setup["minimum_duration_s"] - 1e-12)
            self.assertGreaterEqual(
                setup["actual_steering_duration_s"],
                setup["minimum_steering_duration_s"] - 1e-12)
            self.assertLessEqual(
                setup["steer_rate_peak_radps"],
                setup["steer_rate_limit_radps"] * (1.0 + 1e-9))
            self.assertLessEqual(
                setup["steer_accel_peak_radps2"],
                setup["steer_accel_limit_radps2"] * (1.0 + 1e-9))
        self.assertLessEqual(metrics.get("dynamic_peak_ratio", 0.0),
                             1.0 + ACCEPTANCE_THRESHOLDS[
                                 "dynamic_peak_ratio_s9"])
        taus = metrics.get("dynamic_candidate_taus") or []
        self.assertEqual(metrics.get("dynamic_candidate_count"), 803)
        self.assertEqual(len(taus), 803)
        self.assertEqual(taus, sorted(set(taus)))
        for critical in (
                0.0, 0.5, 0.5 - math.sqrt(3.0) / 6.0,
                0.5 + math.sqrt(3.0) / 6.0, 1.0):
            self.assertIn(critical, taus)
        method = metrics.get("dynamic_method") or {}
        self.assertEqual(method.get("candidate_count"), 803)
        self.assertEqual(
            method.get("strict_continuous_certificate"),
            metrics.get("dynamic_continuous_certificate"))
        quintic = method.get("quintic_supremum") or {}
        self.assertEqual(
            quintic.get("lambda_dot_over_length_per_duration"), 1.875)
        self.assertAlmostEqual(
            quintic.get("lambda_ddot_over_length_per_duration_squared"),
            10.0 / math.sqrt(3.0), places=15)
        self.assertEqual(
            quintic.get("lambda_dddot_over_length_per_duration_cubed"),
            60.0)
        for boundary in metrics.get("boundary_errors", ()):
            self.assertLessEqual(
                boundary["max_heading_or_center_jump_m_or_rad"],
                ACCEPTANCE_THRESHOLDS["time_continuity_s"])
        for phase in plan.phases:
            count = metrics["interior_counts"].get(phase.phase_id)
            self.assertIsNotNone(count)
            self.assertGreaterEqual(
                count, 11,
                "{} has {} interior times (< 11)".format(
                    phase.phase_id, count))
        if plan.edges:
            policy = metrics.get("sweep_policy")
            self.assertIsNotNone(policy)
            self.assertEqual(policy["max_corner_step_m"], 0.01)
            self.assertEqual(policy["motion_padding_m"], 0.01)
            self.assertEqual(policy["max_samples_per_segment"], 200000)
            self.assertEqual(policy["collision_margin_m"], 0.05)
            coverage = metrics.get("sweep_coverage") or {}
            self.assertEqual(set(coverage),
                             {phase.edge_index for phase in plan.phases
                              if isinstance(phase, MotionTiming)})
            for entry in coverage.values():
                self.assertEqual(entry["budget"][0]["max_corner_step_m"],
                                 0.01)
                self.assertEqual(
                    entry["budget"][0]["max_samples_per_segment"], 200000)
                self.assertLessEqual(
                    entry["budget"][0]["sample_count"],
                    entry["budget"][0]["max_samples_per_segment"])
                self.assertGreaterEqual(entry["checked_samples"], 2)
        self.assert_dynamic_confirmation(metrics)
        if clone_result is not None:
            self.assertTrue(clone_result.feasible, clone_result.detail)
            self.assertEqual(clone_result.status, FEASIBLE)
            self.assertEqual(clone_result.reason, "OK")
            self.assertEqual(
                result.metrics.get("model_sha256"),
                clone_result.metrics.get("model_sha256"))
            self.assert_dynamic_confirmation(clone_result.metrics)

    def assert_dynamic_confirmation(self, metrics):
        """Every FEASIBLE result has a completed full-interval bound."""
        tolerance = 1.0 + ACCEPTANCE_THRESHOLDS["dynamic_peak_ratio_s9"]
        self.assertTrue(metrics.get("dynamic_continuous_certificate"))
        self.assertEqual(metrics.get("dynamic_limitations"), [])
        self.assertLessEqual(
            metrics.get("dynamic_bound_ratio", float("inf")), tolerance)
        confirmation = metrics.get("dynamic_confirmation") or {}
        self.assertTrue(confirmation.get("all_constraints_confirmed"))
        coverage = confirmation.get("endpoint_coverage") or {}
        self.assertTrue(coverage.get("tau_start_present"))
        self.assertTrue(coverage.get("tau_end_present"))
        self.assertGreater(confirmation.get("max_tau_gap", 0.0), 0.0)
        for edge_index, vehicles in (
                metrics.get("dynamic_evidence") or {}).items():
            for vehicle_id, values in vehicles.items():
                context = "edge{} vehicle{}".format(edge_index, vehicle_id)
                self.assertTrue(values.get("continuous_certificate"), context)
                self.assertGreater(
                    values.get("max_adjacent_time_gap_s", 0.0), 0.0, context)
                vehicle_coverage = values.get("endpoint_coverage") or {}
                self.assertTrue(
                    vehicle_coverage.get("tau_start_present"), context)
                self.assertTrue(
                    vehicle_coverage.get("tau_end_present"), context)
                self.assertEqual(
                    set(values.get("confirmation") or {}),
                    set(values.get("peaks") or {}), context)
                self.assertEqual(
                    set(values.get("bound_ratios") or {}),
                    set(values["confirmation"]), context)
                self.assertEqual(
                    set(values.get("adopted_bound_ratios") or {}),
                    set(values["confirmation"]), context)
                for name, item in values["confirmation"].items():
                    label = "{} {}".format(context, name)
                    self.assertTrue(item.get("confirmed"), label)
                    self.assertLessEqual(
                        item["adopted_bound"],
                        item["limit"] * tolerance + 1e-15, label)
                    self.assertLessEqual(
                        item["candidate_peak"],
                        item["adopted_bound"] + 1e-15, label)
                    self.assertGreaterEqual(item["limit"], 0.0, label)
                    self.assertAlmostEqual(
                        values["bound_ratios"][name],
                        item["analytic_bound"] / item["limit"], places=12)
                    self.assertAlmostEqual(
                        values["adopted_bound_ratios"][name],
                        item["adopted_bound_ratio"], places=12)
                    self.assertIn(item.get("method"), (
                        "analytic_supremum",
                        "candidate_plus_derivative_lipschitz"), label)
                    self.assertEqual(
                        item.get("analytic_bound_exceeds_limit"),
                        item["analytic_bound"] > item["limit"] * tolerance,
                        label)
                    if item.get("derivative_constraint") is not None:
                        self.assertIsNotNone(
                            item.get("derivative_bound"), label)
                        self.assertIsNotNone(item.get("refined_bound"), label)
                        self.assertIsNotNone(item.get("lipschitz_error"), label)
                        self.assertGreaterEqual(
                            item["refined_bound"],
                            item["candidate_peak"] - 1e-12, label)
                        self.assertGreaterEqual(
                            item["derivative_bound"], 0.0, label)

    # -- B01 ---------------------------------------------------------------
    def test_B01_legal_plans_and_model_roundtrip(self):
        with record_case(
                "B01", "all legal A/O/S plans, zero-angle S, reverse and hold "
                "pass the full audit before and after model roundtrip",
                {"offsets": OFFSETS, "body_shapes": {1: [0.738, 0.500]},
                 "fixtures": [name for name, _ in LEGAL_FIXTURES]}) as record:
            for name, factory in LEGAL_FIXTURES:
                plan = factory()
                projection = plan.sample()
                result = audit(plan, projection)
                clone = clone_plan(plan)
                json_clone = UniformTimedPlan.from_model_json(
                    plan.to_model_json())
                clone_projection = clone.sample()
                clone_result = audit(clone, clone_projection)
                self.assertEqual(plan.model_sha256(), clone.model_sha256())
                self.assertEqual(plan.model_sha256(), json_clone.model_sha256())
                # The factory metadata may reconstruct an angle through a
                # subtraction (e.g. -0.17 -> -0.17000000000000004); the
                # authoritative timed states must still agree exactly.
                roundtrip_error = 0.0
                probe_times = ([0.0] if plan.duration_s == 0.0 else
                               [fraction * plan.duration_s
                                for fraction in
                                (0.1, 0.25, 0.5, 0.75, 0.9)])
                for time_s in probe_times:
                    original_state = plan.state_at(time_s)
                    for other in (
                            clone.state_at(time_s),
                            json_clone.state_at(time_s)):
                        roundtrip_error = max(
                            roundtrip_error,
                            abs(original_state.qf.x - other.qf.x),
                            abs(original_state.qf.y - other.qf.y),
                            abs(original_state.qf.psi - other.qf.psi))
                        for before, after in zip(
                                original_state.vehicles,
                                other.vehicles):
                            roundtrip_error = max(
                                roundtrip_error,
                                abs(before.x - after.x),
                                abs(before.y - after.y),
                                abs(before.theta - after.theta),
                                abs(before.center_speed_mps
                                    - after.center_speed_mps))
                self.assertLessEqual(roundtrip_error, 1e-12)
                self.assert_legal(plan, result, clone_result)
                self.assertEqual(clone_result.reason, result.reason)
                values = dict(
                    fixture=name,
                    model_input_file=write_model_artifact(name, plan),
                    duration_s=plan.duration_s,
                    phase_count=len(plan.phases),
                    model_sha256=plan.model_sha256(),
                    model_hash_roundtrip_equal=True,
                    authoritative_state_roundtrip_error=roundtrip_error,
                    conclusion_before=legal_metrics(result),
                    conclusion_after=legal_metrics(clone_result),
                )
                if name == "S_spin_with_zero_angle_vehicle":
                    motion = next(phase for phase in plan.phases
                                  if isinstance(phase, MotionTiming))
                    state = plan.state_at(
                        motion.t0_s + 0.43 * motion.duration_s)
                    car2 = state.vehicle(2)
                    self.assertEqual(car2.center_speed_mps, 0.0)
                    self.assertEqual(car2.wheel_speed_mps, 0.0)
                    self.assertTrue(all(
                        abs(value) <= 0.0
                        for value in car2.wheel_angular_rates))
                    canonical_angles, canonical_factors = (
                        wheel_configuration(
                            Action(SPIN, 0.0, 0.0), plan.motion_limits))
                    angles = dict(plan.edges[0].wheel_angles_start)[2]
                    factors = dict(plan.edges[0].wheel_factors_start)[2]
                    for actual, expected in zip(angles, canonical_angles):
                        self.assertAlmostEqual(actual, expected, places=12)
                    for actual, expected in zip(factors, canonical_factors):
                        self.assertAlmostEqual(actual, expected, places=12)
                    values.update(
                        zero_angle_vehicle_canonical=True,
                        zero_angle_state={
                            "time_s": state.time_s,
                            "center_speed_mps": car2.center_speed_mps,
                            "wheel_speed_mps": car2.wheel_speed_mps,
                            "direction": car2.direction,
                        })
                if name == "zero_motion_hold":
                    self.assertEqual(plan.edges, ())
                    self.assertEqual(plan.phases, ())
                    self.assertEqual(plan.duration_s, 0.0)
                    values["no_motion_event"] = True
                passed(record, name, **values)

    # -- B02 ---------------------------------------------------------------
    def test_B02_heading_direction_curvature_speed_tamper(self):
        with record_case(
                "B02", "off-tangent body headings, direction, curvature and O "
                "signed-speed projection are each rejected",
                {"arc_start_headings": {1: 0.10, 2: -0.20, 3: 0.30},
                 "heading_shift_rad": 0.01, "curvature_shift": 0.01,
                 "speed_shift_mps": 0.1}) as record:
            # A coherent heading shift: the centre path is untouched, and the
            # tampered heading is stored consistently in the plan and edge.
            plan = arc_plan()
            original = plan.sample()
            self.assertTrue(audit(plan, original).feasible)
            tampered = clone_plan(plan)
            shift = 0.01
            start = dict(tampered.start_headings)
            end = dict(tampered.goal_headings)
            start[2] += shift
            end[2] += shift
            _set(tampered, "start_headings", heading_pairs(start))
            _set(tampered, "goal_headings", heading_pairs(end))
            edge = tampered.edges[0]
            _set(edge, "start_headings", heading_pairs(start))
            _set(edge, "end_headings", heading_pairs(end))
            segments = []
            for segment in edge.segments:
                segment = dataclasses.replace(segment)
                if segment.vehicle_id == 2:
                    _set(segment, "theta_start", start[2])
                    _set(segment, "theta_end", end[2])
                segments.append(segment)
            _set(edge, "segments", tuple(segments))
            original_state = plan.state_at(0.37 * plan.duration_s)
            tampered_state = tampered.state_at(0.37 * tampered.duration_s)
            centre_identical = all(
                abs(a.x - b.x) <= 1e-12 and abs(a.y - b.y) <= 1e-12
                for a, b in zip(original_state.vehicles,
                                tampered_state.vehicles))
            self.assertTrue(centre_identical)
            self.assertGreater(
                abs(original_state.vehicle(2).theta
                    - tampered_state.vehicle(2).theta), 1e-3)
            result = audit(tampered, original)
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH,
                field_contains="start_heading", vehicle_id=2)
            self.assertAlmostEqual(result.failure.max_error, shift, places=15)
            passed(record, "coherent_heading_off_tangent",
                   centre_path_preserved=centre_identical,
                   heading_shift_rad=shift,
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            # Direction tamper: an S delta now has the opposite sign.
            plan = spin_plan()
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            values = dict(edge.directions)
            values[1] = -values[1]
            _set(edge, "directions", heading_pairs(values))
            result = audit(tampered, plan.sample())
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH, vehicle_id=1,
                constraint="direction")
            passed(record, "spin_direction_tamper",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            # Curvature projection tamper on the A arc.
            plan = arc_plan()
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            values = dict(edge.curvatures)
            values[2] += 0.01
            _set(edge, "curvatures", heading_pairs(values))
            result = audit(tampered, plan.sample())
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH,
                field_contains="curvatures", vehicle_id=2)
            passed(record, "arc_curvature_tamper",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            # O signed progress-speed projection tamper.
            plan = o_plan(False)
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            values = dict(edge.signed_progress_speeds)
            values[3] += 0.1
            _set(edge, "signed_progress_speeds", heading_pairs(values))
            result = audit(tampered, plan.sample())
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH,
                field_contains="signed_progress_speeds", vehicle_id=3)
            passed(record, "O_signed_speed_tamper",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

    # -- B03 ---------------------------------------------------------------
    def test_B03_mode_conflict_canonical_and_nan_tamper(self):
        with record_case(
                "B03", "global/segment mode conflicts, S canonical wheels, "
                "wheel factors/IDs and NaN are rejected; legal spin passes",
                {"spin_deltas": {1: math.pi / 2.0, 2: 0.0,
                                 3: -math.pi / 4.0}}) as record:
            legal = spin_plan()
            legal_result = audit(legal, legal.sample())
            self.assert_legal(legal, legal_result)
            passed(record, "legal_spin_positive_control",
                   metrics=legal_metrics(legal_result))

            plan = straight_plan()
            tampered = clone_plan(plan)
            _set(tampered.edges[0], "mode", "O")
            result = audit(tampered, plan.sample())
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH, field_contains="mode")
            passed(record, "global_mode_conflict",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            segments = []
            for segment in edge.segments:
                segment = dataclasses.replace(segment)
                if segment.vehicle_id == 1:
                    _set(segment, "mode", ACKERMANN)
                segments.append(segment)
            _set(edge, "segments", tuple(segments))
            result = audit(tampered, plan.sample())
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH,
                field_contains="segments", vehicle_id=1)
            passed(record, "segment_mode_conflict",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            # The legal zero-angle S vehicle must keep canonical S factors;
            # replacing its start/end angles with A zero-curvature angles is
            # rejected.
            plan = spin_plan()
            edge = plan.edges[0]
            ackermann_angles, _ = wheel_configuration(
                Action(ACKERMANN, 0.0, 0.0), edge.limits)
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            for name in ("wheel_angles_start", "wheel_angles_end"):
                records = []
                for vehicle_id, values in getattr(edge, name):
                    if vehicle_id == 2:
                        values = tuple(float(value)
                                       for value in ackermann_angles)
                    records.append((vehicle_id, tuple(values)))
                _set(edge, name, tuple(records))
            result = audit(tampered, plan.sample())
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH,
                field_contains="wheel_angles_start", vehicle_id=2)
            passed(record, "zero_angle_S_replaced_by_A_wheels",
                   ackermann_angles=list(ackermann_angles),
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            records = []
            for vehicle_id, values in edge.wheel_factors_start:
                records.append((vehicle_id, tuple(
                    value * 2.0 for value in values)
                    if vehicle_id == 1 else values))
            _set(edge, "wheel_factors_start", tuple(records))
            result = audit(tampered, plan.sample())
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH,
                field_contains="wheel_factors_start", vehicle_id=1)
            passed(record, "wheel_factor_tamper",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            records = []
            for vehicle_id, values in edge.wheel_angles_start:
                swapped = {2: 3, 3: 2}.get(vehicle_id, vehicle_id)
                records.append((swapped, tuple(values)))
            _set(edge, "wheel_angles_start", tuple(records))
            result = audit(tampered, plan.sample())
            self.assert_rejected(result, UNIFORM_MODEL_MISMATCH)
            passed(record, "wheel_id_tamper",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = straight_plan()
            original = plan.sample()
            tampered = clone_plan(plan)
            edge = tampered.edges[0]
            values = dict(edge.signed_progress_speeds)
            values[1] = float("nan")
            _set(edge, "signed_progress_speeds", heading_pairs(values))
            result = audit(tampered, original)
            self.assert_rejected(
                result, UNIFORM_MODEL_MISMATCH,
                field_contains="signed_progress_speeds.1")
            self.assertIn("actual", result.failure.field)
            passed(record, "NaN_tamper",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

    # -- B04 ---------------------------------------------------------------
    def _reschedule(self, plan, index, duration):
        phases = list(plan.phases)
        phase = phases[index]
        if isinstance(phase, SetupTiming):
            replacement = dataclasses.replace(
                phase, duration_s=duration,
                steering_duration_s=duration - phase.setup_overhead_s)
        else:
            replacement = dataclasses.replace(phase, duration_s=duration)
        phases[index] = replacement
        cursor = replacement.end_time
        for later in range(index + 1, len(phases)):
            phases[later] = dataclasses.replace(
                phases[later], t0_s=cursor)
            cursor = phases[later].end_time
        _set(plan, "phases", tuple(phases))
        _set(plan, "duration_s", cursor)

    def test_B04_schedule_and_setup_tamper(self):
        with record_case(
                "B04", "shortened motion/setup, deleted setup, omitted demand, "
                "time-axis gap/overlap and early mode switch are rejected",
                {"motion_scale": 0.5, "setup_scale": 0.5,
                 "time_gap_s": 5e-10}) as record:
            plan = straight_plan()
            tampered = clone_plan(plan)
            for index, phase in enumerate(tampered.phases):
                replacement = dataclasses.replace(
                    phase, t0_s=phase.t0_s * 0.5,
                    duration_s=phase.duration_s * 0.5)
                _set(tampered, "phases", tuple(
                    replacement if i == index else item
                    for i, item in enumerate(tampered.phases)))
            _set(tampered, "duration_s", plan.duration_s * 0.5)
            result = audit(tampered, tampered.sample())
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="duration_s", vehicle_id=1,
                constraint="speed")
            self.assertGreater(result.failure.max_error, 0.0)
            passed(record, "shorten_motion_exceeds_speed",
                   original_duration_s=plan.duration_s,
                   tampered_duration_s=tampered.duration_s,
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            tampered = clone_plan(plan)
            setup = tampered.phases[0]
            self.assertIsInstance(setup, SetupTiming)
            self._reschedule(tampered, 0, setup.duration_s * 0.5)
            result = audit(tampered, tampered.sample())
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="duration_s")
            self.assertEqual(result.failure.block_index, 0)
            passed(record, "shorten_initial_setup",
                   original_setup_s=setup.duration_s,
                   tampered_setup_s=tampered.phases[0].duration_s,
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = complex_plan()
            tampered = clone_plan(plan)
            index = next(i for i, phase in enumerate(tampered.phases)
                         if isinstance(phase, SetupTiming)
                         and not phase.initial)
            setup = tampered.phases[index]
            self._reschedule(tampered, index, setup.duration_s * 0.5)
            result = audit(tampered, tampered.sample())
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="duration_s")
            self.assertEqual(result.failure.block_index,
                             setup.edge_index)
            passed(record, "shorten_adjacent_setup",
                   phase_id=setup.phase_id,
                   original_setup_s=setup.duration_s,
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            tampered = clone_plan(plan)
            phases = [phase for phase in tampered.phases
                      if not isinstance(phase, SetupTiming)]
            phases[0] = dataclasses.replace(phases[0], t0_s=0.0)
            _set(tampered, "phases", tuple(phases))
            _set(tampered, "duration_s", phases[0].duration_s)
            result = audit(tampered, tampered.sample())
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="phases")
            passed(record, "delete_initial_setup",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            original = plan.sample()
            tampered = clone_plan(plan)
            setup = tampered.phases[0]
            self.assertIsInstance(setup, SetupTiming)
            demands = tuple(demand for demand in setup.demands
                            if demand.vehicle_id != 1)
            self.assertEqual(len(demands), len(setup.demands) - 1)
            _set(setup, "demands", demands)
            result = audit(tampered, original)
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="demands", vehicle_id=1)
            passed(record, "omit_slowest_vehicle_demand",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = straight_plan()
            original = plan.sample()
            tampered = clone_plan(plan)
            motion = tampered.phases[0]
            _set(tampered, "phases", (dataclasses.replace(
                motion, t0_s=5e-10),))
            result = audit(tampered, original)
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="t0_s")
            passed(record, "time_axis_gap", gap_s=5e-10,
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            original = plan.sample()
            tampered = clone_plan(plan)
            phases = list(tampered.phases)
            phases[1] = dataclasses.replace(
                phases[1], t0_s=phases[1].t0_s - 5e-10)
            _set(tampered, "phases", tuple(phases))
            result = audit(tampered, original)
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="t0_s")
            passed(record, "time_axis_overlap", overlap_s=5e-10,
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

            plan = spin_plan()
            tampered = clone_plan(plan)
            setup = tampered.phases[0]
            demands = []
            for demand in setup.demands:
                if demand.vehicle_id == 2:
                    demand = copy.copy(demand)
                    _set(demand, "to_mode", "O")
                demands.append(demand)
            _set(setup, "demands", tuple(demands))
            result = audit(tampered, tampered.sample())
            self.assert_rejected(
                result, UNIFORM_PHASE_SCHEDULE_INVALID,
                field_contains="demands.to_mode", vehicle_id=2)
            passed(record, "early_mode_switch",
                   tamper=tamper_evidence("plan", plan, tampered),
                   rejection=failure_evidence(result))

    # -- B05 ---------------------------------------------------------------
    def test_B05_projection_only_tamper(self):
        with record_case(
                "B05", "projection-only changes and rigid projection tampering "
                "are rejected while the model is unchanged",
                {"fixture": "A_arc_nonzero_pivot"}) as record:
            plan = arc_plan()
            model_file = write_model_artifact(
                "A_arc_nonzero_pivot", plan)
            projection = plan.sample()
            self.assertTrue(audit(plan, projection).feasible)
            model_hash = plan.model_sha256()
            middle = len(projection.times) // 2

            def state_tamper(transform):
                tampered = clone_projection(projection)
                states = list(tampered.states)
                states[middle] = transform(states[middle])
                _set(tampered, "states", tuple(states))
                return tampered

            def reject_projection(name, tampered, field_contains):
                self.assertEqual(plan.model_sha256(), model_hash)
                self.assertTrue(audit(plan, projection).feasible)
                result = audit(plan, tampered)
                self.assert_rejected(
                    result, UNIFORM_PROJECTION_MISMATCH,
                    field_contains=field_contains)
                passed(record, name, model_unchanged=True,
                       model_input_file=model_file,
                       tamper=tamper_evidence(
                           "projection", projection, tampered),
                       rejection=failure_evidence(result))

            reject_projection(
                "projection_vehicle_x",
                state_tamper(lambda state: dataclasses.replace(
                    state, vehicles=tuple(
                        dataclasses.replace(vehicle, x=vehicle.x + 0.01)
                        if vehicle.vehicle_id == 2 else vehicle
                        for vehicle in state.vehicles))),
                "projection.state")
            reject_projection(
                "projection_qf_psi",
                state_tamper(lambda state: dataclasses.replace(
                    state, qf=FormationPose(
                        state.qf.x, state.qf.y, state.qf.psi + 0.001))),
                "projection.state")
            reject_projection(
                "projection_qf_position",
                state_tamper(lambda state: dataclasses.replace(
                    state, qf=FormationPose(
                        state.qf.x + 0.01, state.qf.y, state.qf.psi))),
                "projection.state")
            reject_projection(
                "projection_state_time",
                state_tamper(lambda state: dataclasses.replace(
                    state, time_s=state.time_s + 1e-6)),
                "projection.states.time_s")

            timed = clone_timed
            for name, field, transform, contains in (
                    ("projection_timed_center", "center_x",
                     lambda point: dataclasses.replace(
                         point, center_x=point.center_x + 0.01),
                     "timed.point"),
                    ("projection_timed_theta", "theta",
                     lambda point: dataclasses.replace(
                         point, theta=point.theta + 0.001),
                     "timed.point"),
                    ("projection_timed_theta_unwrapped", "theta",
                     lambda point: dataclasses.replace(
                         point, theta=point.theta + 2.0 * math.pi),
                     "timed.point"),
                    ("projection_timed_speed", "center_speed_mps",
                     lambda point: dataclasses.replace(
                         point,
                         center_speed_mps=point.center_speed_mps + 0.1),
                     "timed.point"),
                    ("projection_timed_mode", "mode",
                     lambda point: dataclasses.replace(point, mode=SPIN),
                     "timed.mode")):
                tampered = clone_projection(projection)
                trajectory = timed(tampered)
                points = list(trajectory.trajectories[1])
                points[middle] = transform(points[middle])
                trajectories = dict(trajectory.trajectories)
                trajectories[1] = tuple(points)
                _set(trajectory, "trajectories", trajectories)
                reject_projection(name, tampered, contains)

            tampered = clone_projection(projection)
            trajectory = timed(tampered)
            poses = list(trajectory.formation_poses)
            poses[middle] = dataclasses.replace(
                poses[middle], psi=poses[middle].psi + 0.001)
            _set(trajectory, "formation_poses", tuple(poses))
            reject_projection(
                "projection_formation_pose_psi", tampered,
                "timed.formation_poses")

            # Rigid translation of the entire projection: every centre keeps
            # its rigid offset from q_F, but the model does not translate.
            delta = 0.01
            tampered = clone_projection(projection)
            states = list(tampered.states)
            for index, state in enumerate(states):
                states[index] = dataclasses.replace(
                    state, qf=FormationPose(
                        state.qf.x + delta, state.qf.y, state.qf.psi),
                    vehicles=tuple(
                        dataclasses.replace(vehicle, x=vehicle.x + delta)
                        for vehicle in state.vehicles))
            _set(tampered, "states", tuple(states))
            trajectory = timed(tampered)
            _set(trajectory, "formation_poses", tuple(
                FormationPose(qf.x + delta, qf.y, qf.psi)
                for qf in trajectory.formation_poses))
            for vehicle_id in sorted(trajectory.trajectories):
                points = list(trajectory.trajectories[vehicle_id])
                for index, point in enumerate(points):
                    points[index] = dataclasses.replace(
                        point, center_x=point.center_x + delta)
                trajectories = dict(trajectory.trajectories)
                trajectories[vehicle_id] = tuple(points)
                _set(trajectory, "trajectories", trajectories)
            rigidity_error = 0.0
            for state in tampered.states:
                cosine, sine = math.cos(state.qf.psi), math.sin(state.qf.psi)
                for vehicle in state.vehicles:
                    offset = next(offset for offset in plan.model.offsets
                                  if offset.vehicle_id == vehicle.vehicle_id)
                    expected_x = state.qf.x + cosine * offset.x - sine * offset.y
                    expected_y = state.qf.y + sine * offset.x + cosine * offset.y
                    rigidity_error = max(
                        rigidity_error, abs(vehicle.x - expected_x),
                        abs(vehicle.y - expected_y))
            self.assertLessEqual(rigidity_error, 1e-6)
            reject_projection(
                "projection_rigid_translation", tampered,
                "projection.state")

            # Formation chord interpolation at an interior sample.
            tampered = clone_projection(projection)
            trajectory = timed(tampered)
            poses = list(trajectory.formation_poses)
            first, last = poses[middle - 1], poses[middle + 1]
            poses[middle] = FormationPose(
                (first.x + last.x) / 2.0, (first.y + last.y) / 2.0,
                (first.psi + last.psi) / 2.0)
            _set(trajectory, "formation_poses", tuple(poses))
            reject_projection(
                "projection_formation_chord", tampered,
                "timed.formation_poses")

    # -- B06 ---------------------------------------------------------------
    def _obstacle_grid(self, point, half_cells=1):
        grid = Grid(np.zeros((400, 400), dtype=bool), 0.05, -10.0, -10.0)
        ix = int(math.floor((point[0] - grid.ox) / grid.res))
        iy = int(math.floor((point[1] - grid.oy) / grid.res))
        cells = []
        for row in range(iy - half_cells, iy + half_cells + 1):
            for column in range(ix - half_cells, ix + half_cells + 1):
                grid.occ[row, column] = True
                cells.append([row, column])
        return grid, cells

    def _checker(self, grid, plan, margin=0.05):
        return FormationCollisionChecker(
            grid, plan.model, BODY_SHAPES, margin)

    def _assert_endpoints_clear(self, grid, plan):
        checker = self._checker(grid, plan)
        reports = (
            ("start", plan.start_qf, dict(plan.start_headings)),
            ("goal", plan.goal_qf, dict(plan.goal_headings)),
        )
        evidence = []
        for name, qf, body_headings in reports:
            report = checker.static_check(qf, body_headings)
            self.assertTrue(report.success,
                            "{} formation collides: {}".format(
                                name, report.detail))
            evidence.append({"endpoint": name, "success": True,
                             "reason": report.reason})
        return evidence

    def _assert_first_collision(self, plan, result, grid):
        self.assert_rejected(
            result, FORMATION_SWEEP_COLLISION,
            field_contains="collision")
        collision = result.metrics.get("first_collision")
        self.assertIsNotNone(collision)
        time_s = collision["time_s"]
        station = collision["station"]
        self.assertGreater(time_s, 0.0)
        self.assertLess(time_s, plan.duration_s)
        self.assertGreater(station, 0.0)
        self.assertLess(station, 1.0)
        state = plan.state_at(time_s)
        reported_qf = collision["qf"]
        self.assertIsNotNone(reported_qf)
        qf_error = max(
            abs(reported_qf[index] - value)
            for index, value in enumerate(
                (state.qf.x, state.qf.y, state.qf.psi)))
        self.assertLessEqual(
            qf_error, 1e-9,
            "reported collision qf {} is not the analytic state {} "
            "(error {})".format(reported_qf, state.qf, qf_error))
        self.assertEqual(collision["label"],
                         next(phase.phase_id for phase in plan.phases
                              if isinstance(phase, MotionTiming)
                              and phase.edge_index
                              == collision["edge_index"]))
        self.assertEqual(result.metrics.get("map_signature"),
                         grid_signature(grid))
        collision["state_recompute_error_m_or_rad"] = qf_error
        return collision, state

    def test_B06_analytic_sweep_mid_motion_collision(self):
        with record_case(
                "B06", "endpoint-clear maps with mid-sweep collisions are "
                "rejected with the exact analytic first collision state",
                {"straight_goal": [6.0, 0.0],
                 "obstacle_straight": [4.2, 0.0],
                 "fixture_arc": "S_A_S_O_complex"}) as record:
            plan = straight_plan(6.0)
            straight6_file = write_model_artifact(
                "B06_straight_goal_6m", plan)
            empty_grid = Grid(
                np.zeros((400, 400), dtype=bool), 0.05, -10.0, -10.0)
            empty_result = audit(plan, plan.sample(), empty_grid)
            self.assertTrue(empty_result.feasible)
            grid, cells = self._obstacle_grid((4.2, 0.0))
            endpoints = self._assert_endpoints_clear(grid, plan)
            result = audit(plan, plan.sample(), grid)
            collision, state = self._assert_first_collision(
                plan, result, grid)
            self.assertEqual(collision["vehicle_id"], 1)
            passed(record, "straight_mid_sweep_collision",
                   model_input_file=straight6_file,
                   map_change={
                       "before_signature": grid_signature(empty_grid),
                       "after_signature": grid_signature(grid),
                       "before_status": str(empty_result.status),
                       "after_status": str(result.status),
                       "changed_cells": cells,
                   },
                   grid={"resolution": grid.res, "origin": [grid.ox, grid.oy],
                         "shape": [grid.height, grid.width],
                         "length": grid.half_l * 2 - 0.04,
                         "width": grid.half_w * 2 - 0.04,
                         "margin": 0.04},
                   obstacle_cells=cells,
                   endpoint_checks=endpoints,
                   first_collision=collision,
                   analytic_state=state.as_dict(),
                   endpoint_only_detection_would_miss=True,
                   rejection=failure_evidence(result))

            plan = complex_plan()
            complex_file = write_model_artifact("S_A_S_O_complex", plan)
            projection = plan.sample()
            arc_phase = next(
                phase for phase in plan.phases
                if isinstance(phase, MotionTiming)
                and phase.edge.primitive == "A_ARC")
            collision_time = (
                arc_phase.t0_s + 0.5 * arc_phase.duration_s)
            mid_state = plan.state_at(collision_time)
            vehicle = mid_state.vehicle(1)
            empty_grid = Grid(
                np.zeros((400, 400), dtype=bool), 0.05, -10.0, -10.0)
            empty_result = audit(plan, projection, empty_grid)
            self.assertTrue(empty_result.feasible)
            grid, cells = self._obstacle_grid((vehicle.x, vehicle.y))
            endpoints = self._assert_endpoints_clear(grid, plan)
            result = audit(plan, projection, grid)
            collision, state = self._assert_first_collision(
                plan, result, grid)
            self.assertEqual(collision["vehicle_id"], 1)
            self.assertLess(collision["station"], 1.0)
            self.assertGreater(
                abs(collision["station"] - 0.5), 0.05,
                "the first collision should not be the obstacle midpoint "
                "sample itself")
            passed(record, "nonzero_pivot_arc_mid_sweep_collision",
                   model_input_file=complex_file,
                   map_change={
                       "before_signature": grid_signature(empty_grid),
                       "after_signature": grid_signature(grid),
                       "before_status": str(empty_result.status),
                       "after_status": str(result.status),
                       "changed_cells": cells,
                   },
                   grid={"resolution": grid.res, "origin": [grid.ox, grid.oy],
                         "shape": [grid.height, grid.width],
                         "length": grid.half_l * 2 - 0.04,
                         "width": grid.half_w * 2 - 0.04,
                         "margin": 0.04},
                   obstacle_cells=cells,
                   obstacle_center=[vehicle.x, vehicle.y],
                   obstacle_center_time_s=collision_time,
                   endpoint_checks=endpoints,
                   first_collision=collision,
                   analytic_state=state.as_dict(),
                   endpoint_only_detection_would_miss=True,
                   rejection=failure_evidence(result))

            # Spin in place: both endpoint formations are clear, but a single
            # occupied cell sweeps through car 1 near the middle of the spin.
            plan = spin_plan()
            spin_file = write_model_artifact("B06_spin_mid_sweep", plan)
            empty_grid = Grid(
                np.zeros((400, 400), dtype=bool), 0.05, -10.0, -10.0)
            empty_result = audit(plan, plan.sample(), empty_grid)
            self.assertTrue(empty_result.feasible)
            grid, cells = self._obstacle_grid((1.075, 0.475), half_cells=0)
            self.assertEqual(cells, [[209, 221]])
            endpoints = self._assert_endpoints_clear(grid, plan)
            result = audit(plan, plan.sample(), grid)
            collision, state = self._assert_first_collision(
                plan, result, grid)
            self.assertEqual(collision["vehicle_id"], 1)
            self.assertAlmostEqual(
                collision["time_s"], 4.088966537921189, places=9)
            self.assertAlmostEqual(
                collision["station"], 0.47183098591549294, places=9)
            self.assertGreater(collision["station"], 0.0)
            self.assertLess(collision["station"], 1.0)
            passed(record, "spin_endpoint_clear_mid_sweep_collision",
                   model_input_file=spin_file,
                   map_change={
                       "before_signature": grid_signature(empty_grid),
                       "after_signature": grid_signature(grid),
                       "before_status": str(empty_result.status),
                       "after_status": str(result.status),
                       "changed_cells": cells,
                   },
                   grid={"resolution": grid.res, "origin": [grid.ox, grid.oy],
                         "shape": [grid.height, grid.width],
                         "length": grid.half_l * 2 - 0.04,
                         "width": grid.half_w * 2 - 0.04,
                         "margin": 0.04},
                   obstacle_cells=cells,
                   endpoint_checks=endpoints,
                   first_collision=collision,
                   analytic_state=state.as_dict(),
                   endpoint_only_detection_would_miss=True,
                   rejection=failure_evidence(result))

    # -- B07 ---------------------------------------------------------------
    def _grid_times(self, duration, dt, extra=()):
        values = [0.0]
        count = int(math.floor(duration / dt))
        values.extend(index * dt for index in range(1, count + 1))
        if duration - values[-1] > 1e-12:
            values.append(duration)
        return sorted(set(values) | set(float(value) for value in extra))

    def test_B07_projection_rate_consistency_and_chord_tamper(self):
        setup_times = [0.137, 0.419, 0.777]
        with record_case(
                "B07", "0.02 s and 0.05 s projections of the same model agree "
                "exactly at common off-grid setup times and inside every "
                "motion phase; chord interpolation is rejected",
                {"fixture": "S_A_S_O_complex", "rates_s": [0.02, 0.05],
                 "setup_offgrid_times_s": setup_times,
                 "motion_relative_fraction_of_phase_duration": 0.371}) as record:
            plan = complex_plan()
            model_file = write_model_artifact("S_A_S_O_complex", plan)
            duration = plan.duration_s
            motion_times = [
                phase.t0_s + 0.371 * phase.duration_s
                for phase in plan.phases if isinstance(phase, MotionTiming)]
            self.assertEqual(len(motion_times), 4)
            common = setup_times + motion_times
            record["inputs"]["common_offgrid_times_s"] = common
            record["inputs"]["motion_offgrid_times_s"] = motion_times
            self.assertEqual(len(common), len(set(common)))
            times_02 = self._grid_times(duration, 0.02, common)
            times_05 = self._grid_times(duration, 0.05, common)
            for value in common:
                self.assertIn(value, times_02)
                self.assertIn(value, times_05)
                self.assertNotAlmostEqual(value / 0.02,
                                          round(value / 0.02), places=9)
                self.assertNotAlmostEqual(value / 0.05,
                                          round(value / 0.05), places=9)
            projection_02 = plan.sample(times_02)
            projection_05 = plan.sample(times_05)
            result_02 = audit(plan, projection_02)
            result_05 = audit(plan, projection_05)
            self.assert_legal(plan, result_02)
            self.assert_legal(plan, result_05)
            motion_phase_by_id = {
                phase.phase_id: phase for phase in plan.phases
                if isinstance(phase, MotionTiming)}
            comparisons = []
            motion_modes = []
            for time_s in common:
                state_02 = projection_02.states[times_02.index(time_s)]
                state_05 = projection_05.states[times_05.index(time_s)]
                analytic = plan.state_at(time_s)
                self.assertEqual(state_02.time_s, time_s)
                self.assertEqual(state_05.time_s, time_s)
                self.assertEqual(state_02.as_dict(), state_05.as_dict())
                self.assertEqual(state_02.as_dict(), analytic.as_dict())
                if time_s in setup_times:
                    self.assertEqual(analytic.phase_kind, "initial_setup")
                else:
                    self.assertEqual(analytic.phase_kind, "motion")
                    self.assertIn(analytic.phase_id, motion_phase_by_id)
                    motion_modes.append(analytic.mode)
                comparisons.append({
                    "time_s": time_s,
                    "phase_id": analytic.phase_id,
                    "phase_kind": analytic.phase_kind,
                    "mode": analytic.mode,
                    "states_bit_identical": True,
                    "qf": [analytic.qf.x, analytic.qf.y, analytic.qf.psi],
                })
            self.assertEqual(motion_modes, ["S", "A", "S", "O"])
            passed(record, "same_model_two_sampling_rates",
                   model_input_file=model_file,
                   duration_s=duration,
                   sample_counts={"0.02": len(times_02),
                                  "0.05": len(times_05)},
                   projections={
                       "0.02": legal_metrics(result_02),
                       "0.05": legal_metrics(result_05),
                   },
                   setup_times_s=setup_times,
                   motion_times_s=motion_times,
                   motion_phase_modes=motion_modes,
                   common_time_comparisons=comparisons,
                   discrete_clearance_values_compared=False,
                   note="clearance grids differ; only exact states are "
                        "compared at common times")

            # Replace one 0.05 s sample by the chord midpoint of its
            # neighbours, and rewrite the rigid formation around that chord
            # pose so the projection stays internally rigid.
            projection = projection_05
            index = next(
                index for index, time_s in enumerate(times_05)
                if arc_phase_active(plan, time_s))
            # Choose a sample near the middle of the A arc motion phase.
            arc_phase = next(
                phase for phase in plan.phases
                if isinstance(phase, MotionTiming)
                and phase.edge.primitive == "A_ARC")
            index = min(
                range(1, len(times_05) - 1),
                key=lambda i: abs(
                    times_05[i] - (arc_phase.t0_s
                                   + 0.5 * arc_phase.duration_s)))
            tampered = clone_projection(projection)
            trajectory = clone_timed(tampered)
            poses = list(trajectory.formation_poses)
            first, last = poses[index - 1], poses[index + 1]
            chord = FormationPose(
                (first.x + last.x) / 2.0, (first.y + last.y) / 2.0,
                (first.psi + last.psi) / 2.0)
            original_chord = poses[index]
            poses[index] = chord
            _set(trajectory, "formation_poses", tuple(poses))
            states = list(tampered.states)
            state = states[index]
            cosine, sine = math.cos(chord.psi), math.sin(chord.psi)
            vehicles = []
            for vehicle in state.vehicles:
                offset = next(offset for offset in plan.model.offsets
                              if offset.vehicle_id == vehicle.vehicle_id)
                vehicles.append(dataclasses.replace(
                    vehicle,
                    x=chord.x + cosine * offset.x - sine * offset.y,
                    y=chord.y + sine * offset.x + cosine * offset.y))
            states[index] = dataclasses.replace(
                state, qf=chord, vehicles=tuple(vehicles))
            _set(tampered, "states", tuple(states))
            trajectories = dict(trajectory.trajectories)
            for vehicle_id in sorted(trajectories):
                offset = next(offset for offset in plan.model.offsets
                              if offset.vehicle_id == vehicle_id)
                points = list(trajectories[vehicle_id])
                points[index] = dataclasses.replace(
                    points[index],
                    center_x=chord.x + cosine * offset.x - sine * offset.y,
                    center_y=chord.y + sine * offset.x + cosine * offset.y)
                trajectories[vehicle_id] = tuple(points)
            _set(trajectory, "trajectories", trajectories)
            rigidity_error = 0.0
            for vehicle in states[index].vehicles:
                offset = next(offset for offset in plan.model.offsets
                              if offset.vehicle_id == vehicle.vehicle_id)
                expected_x = chord.x + cosine * offset.x - sine * offset.y
                expected_y = chord.y + sine * offset.x + cosine * offset.y
                rigidity_error = max(
                    rigidity_error, abs(vehicle.x - expected_x),
                    abs(vehicle.y - expected_y))
            self.assertLessEqual(rigidity_error, 1e-6)
            chord_delta = math.hypot(
                chord.x - original_chord.x, chord.y - original_chord.y)
            self.assertGreater(chord_delta, 1e-6)
            result = audit(plan, tampered)
            self.assert_rejected(
                result, UNIFORM_PROJECTION_MISMATCH,
                field_contains="projection.state")
            self.assertEqual(result.failure.sample_index, index)
            passed(record, "formation_chord_interpolation_rejected",
                   model_input_file=model_file,
                   chord_time_s=times_05[index],
                   chord_delta_m=chord_delta,
                   rigidity_error_m=rigidity_error,
                   tamper=tamper_evidence("projection", projection, tampered),
                   rejection=failure_evidence(result))

    # -- B08 ---------------------------------------------------------------
    def test_B08_version_signature_identity_and_budget(self):
        with record_case(
                "B08", "unknown versions, bad map signature, projection-model "
                "identity mismatch and sweep budget exhaustion fail "
                "explicitly",
                {"model_version_bad": "v9",
                 "map_signature_bad": "deadbeef",
                 "max_samples_per_segment_bad": 1}) as record:
            plan = straight_plan()
            model_file = write_model_artifact("A_straight_forward", plan)
            projection = plan.sample()
            self.assertTrue(audit(plan, projection).feasible)

            tampered = clone_projection(projection)
            _set(tampered, "model_version", "v9")
            result = audit(plan, tampered)
            self.assert_rejected(
                result, UNIFORM_PROJECTION_MISMATCH,
                field_contains="projection.model_version")
            passed(record, "unknown_model_version",
                   model_input_file=model_file,
                   tamper=tamper_evidence(
                       "projection", projection, tampered),
                   rejection=failure_evidence(result))

            tampered = clone_projection(projection)
            _set(tampered, "semantic_version", "fr01.legacy.v1")
            result = audit(plan, tampered)
            self.assert_rejected(
                result, UNIFORM_PROJECTION_MISMATCH,
                field_contains="projection.semantic_version")
            passed(record, "unknown_projection_semantics",
                   model_input_file=model_file,
                   tamper=tamper_evidence(
                       "projection", projection, tampered),
                   rejection=failure_evidence(result))

            default_config = FormationQualityConfig()
            config = FormationQualityConfig(
                expected_map_signature="deadbeef")
            result = audit(plan, projection, config=config)
            self.assert_rejected(
                result, INVALID_REQUEST, field_contains="map_signature")
            self.assertEqual(result.status, INVALID)
            passed(record, "illegal_map_signature",
                   model_input_file=model_file,
                   tamper=tamper_evidence("config", default_config, config),
                   rejection=failure_evidence(result))

            tampered = clone_projection(projection)
            _set(tampered, "plan_id", plan.plan_id + 1)
            result = audit(plan, tampered)
            self.assert_rejected(
                result, UNIFORM_PROJECTION_MISMATCH,
                field_contains="projection.plan_id")
            passed(record, "projection_plan_id_mismatch",
                   model_input_file=model_file,
                   tamper=tamper_evidence(
                       "projection", projection, tampered),
                   rejection=failure_evidence(result))

            tampered = clone_projection(projection)
            _set(tampered, "model_sha256", "0" * 64)
            result = audit(plan, tampered)
            self.assert_rejected(
                result, UNIFORM_PROJECTION_MISMATCH,
                field_contains="projection.model_sha256")
            passed(record, "projection_model_hash_mismatch",
                   model_input_file=model_file,
                   tamper=tamper_evidence(
                       "projection", projection, tampered),
                   rejection=failure_evidence(result))

            tampered = clone_projection(projection)
            _set(tampered, "certification", "LEGACY_OK")
            result = audit(plan, tampered)
            self.assert_rejected(
                result, UNIFORM_PROJECTION_MISMATCH,
                field_contains="projection.certification")
            passed(record, "projection_certification_mismatch",
                   model_input_file=model_file,
                   tamper=tamper_evidence(
                       "projection", projection, tampered),
                   rejection=failure_evidence(result))

            config = FormationQualityConfig(max_samples_per_segment=1)
            result = audit(plan, projection, config=config)
            self.assert_rejected(
                result, UNIFORM_SWEEP_BUDGET_EXCEEDED,
                field_contains="sweep_budget")
            self.assertEqual(result.status, INVALID)
            self.assertNotEqual(result.reason, FORMATION_RETIMING_INFEASIBLE)
            self.assertEqual(result.failure.block_index, 0)
            self.assertEqual(result.metrics.get("sweep_policy"), {
                "max_corner_step_m": 0.01,
                "max_samples_per_segment": 1,
                "motion_padding_m": 0.01,
                "collision_margin_m": 0.05,
            })
            passed(record, "sweep_budget_exhaustion",
                   model_input_file=model_file,
                   sweep_policy=result.metrics.get("sweep_policy"),
                   tamper=tamper_evidence("config", default_config, config),
                   rejection=failure_evidence(result))

    # -- B09 ---------------------------------------------------------------
    def test_B09_dynamic_confirmation_and_gate(self):
        with record_case(
                "B09", "every complete FEASIBLE result carries a completed "
                "full-interval dynamic confirmation; an interval that cannot "
                "be confirmed is rejected and cannot be published",
                {"confirmed": ["A_arc_nonzero_pivot", "S_A_S_O_complex"],
                 "unconfirmed": "A_arc_nonzero_pivot with jerk_max=1.5"}) \
                as record:
            plan = arc_plan()
            arc_file = write_model_artifact("B09_arc_confirmed", plan)
            projection = plan.sample()
            result = audit(plan, projection)
            self.assert_legal(plan, result)
            evidence = result.metrics["dynamic_evidence"][0][3]
            acceleration = evidence["confirmation"]["acceleration"]
            self.assertEqual(acceleration["derivative_constraint"], "jerk")
            self.assertEqual(acceleration["method"],
                             "candidate_plus_derivative_lipschitz")
            self.assertTrue(acceleration["analytic_bound_exceeds_limit"])
            self.assertGreater(acceleration["analytic_bound"],
                               acceleration["limit"])
            self.assertGreater(acceleration["candidate_peak"], 0.0)
            self.assertLess(acceleration["candidate_peak"],
                            acceleration["refined_bound"])
            self.assertLess(acceleration["refined_bound"],
                            acceleration["limit"])
            self.assertLess(acceleration["adopted_bound"],
                            acceleration["limit"])
            self.assertEqual(acceleration["adopted_bound"],
                             acceleration["refined_bound"])
            self.assertGreater(acceleration["derivative_bound"], 0.0)
            self.assertAlmostEqual(
                acceleration["refined_bound"],
                acceleration["candidate_peak"]
                + acceleration["derivative_bound"]
                * evidence["max_adjacent_time_gap_s"] / 2.0, places=12)
            gate = FormationPublicationGate()
            gate.begin_request(plan.plan_id)
            decision = gate.offer(
                plan.plan_id, projection.timed_trajectory, result)
            self.assertTrue(decision.publish, decision.detail)
            self.assertEqual(decision.reason, "OK")
            self.assertIs(decision.trajectory,
                          projection.timed_trajectory)
            passed(record, "confirmed_arc_publishes",
                   model_input_file=arc_file,
                   acceleration_confirmation=dict(acceleration),
                   max_adjacent_time_gap_s=evidence[
                       "max_adjacent_time_gap_s"],
                   endpoint_coverage=evidence["endpoint_coverage"],
                   gate=decision.as_status())

            complex_view = complex_plan()
            complex_file = write_model_artifact(
                "B09_complex_confirmed", complex_view)
            complex_projection = complex_view.sample()
            complex_result = audit(complex_view, complex_projection)
            self.assert_legal(complex_view, complex_result)
            motion_indices = {
                phase.edge_index for phase in complex_view.phases
                if isinstance(phase, MotionTiming)}
            evidence_by_edge = complex_result.metrics["dynamic_evidence"]
            self.assertEqual(set(evidence_by_edge), motion_indices)
            confirmed_counts = {}
            for edge_index, vehicles in evidence_by_edge.items():
                self.assertEqual(set(vehicles), {1, 2, 3})
                confirmed_counts[edge_index] = {}
                for vehicle_id, values in vehicles.items():
                    self.assertTrue(values["continuous_certificate"])
                    self.assertEqual(set(values["confirmation"]),
                                     set(values["peaks"]))
                    for name, item in values["confirmation"].items():
                        self.assertTrue(
                            item["confirmed"],
                            "edge{} vehicle{} {}".format(
                                edge_index, vehicle_id, name))
                    confirmed_counts[edge_index][vehicle_id] = sum(
                        1 for item in values["confirmation"].values()
                        if item["confirmed"])
            complex_gate = FormationPublicationGate()
            complex_gate.begin_request(complex_view.plan_id)
            complex_decision = complex_gate.offer(
                complex_view.plan_id, complex_projection.timed_trajectory,
                complex_result)
            self.assertTrue(complex_decision.publish,
                            complex_decision.detail)
            passed(record, "confirmed_complex_publishes",
                   model_input_file=complex_file,
                   motion_edge_indices=sorted(motion_indices),
                   confirmed_constraint_counts=confirmed_counts,
                   confirmation=complex_result.metrics[
                       "dynamic_confirmation"],
                   gate=complex_decision.as_status())

            # The same A arc, retimed by S8 for a tighter jerk limit, has a
            # candidate peak below the limit but an analytic full-interval
            # bound above it.  That is an unconfirmed audit, not an observed
            # exceedance, and it must not reach FEASIBLE or the gate.
            limits = FormationTimingLimits(jerk_max=1.5)
            motion_limits = timing_motion_limits(limits)
            qf = FormationPose(2.0, -1.0, 0.35)
            start_headings = {1: 0.10, 2: -0.20, 3: 0.30}
            pivot = (0.0, 2.0)
            delta = 0.60
            arc_headings = a_arc_required_headings(
                model(), qf, delta, pivot, directions(),
                limits=motion_limits)
            edge = a_arc_edge(
                model(), qf, arc_headings, delta, pivot, directions(),
                limits=motion_limits)
            unconfirmed_plan = plan_for((edge,), limits=limits)
            unconfirmed_projection = unconfirmed_plan.sample()
            unconfirmed_file = write_model_artifact(
                "B09_arc_jerk_unconfirmed", unconfirmed_plan)
            unconfirmed = audit(
                unconfirmed_plan, unconfirmed_projection)
            self.assert_rejected(
                unconfirmed, UNIFORM_DYNAMIC_UNCONFIRMED,
                field_contains="dynamic_confirmation",
                vehicle_id=3, constraint="jerk")
            self.assertEqual(unconfirmed.status, REJECTED)
            self.assertFalse(unconfirmed.feasible)
            self.assertFalse(unconfirmed.executable)
            self.assertNotEqual(
                unconfirmed.reason, FORMATION_RETIMING_INFEASIBLE)
            self.assertFalse(unconfirmed.metrics[
                "dynamic_continuous_certificate"])
            confirmation = unconfirmed.metrics["dynamic_confirmation"]
            self.assertFalse(confirmation["all_constraints_confirmed"])
            self.assertEqual(len(confirmation["unconfirmed"]), 1)
            item = confirmation["unconfirmed"][0]
            self.assertEqual(item["vehicle_id"], 3)
            self.assertEqual(item["constraint"], "jerk")
            self.assertLess(item["candidate_peak"], item["limit"])
            self.assertGreater(item["adopted_bound"], item["limit"])
            self.assertTrue(item["unconfirmed"])
            self.assertEqual(item["endpoint_coverage"], {
                "tau_start_present": True, "tau_end_present": True})
            self.assertGreater(item["max_adjacent_time_gap_s"], 0.0)
            self.assertAlmostEqual(
                unconfirmed.failure.max_error,
                item["adopted_bound"], places=12)
            unconfirmed_gate = FormationPublicationGate()
            unconfirmed_gate.begin_request(unconfirmed_plan.plan_id)
            unconfirmed_decision = unconfirmed_gate.offer(
                unconfirmed_plan.plan_id,
                unconfirmed_projection.timed_trajectory, unconfirmed)
            self.assertFalse(unconfirmed_decision.publish)
            self.assertEqual(unconfirmed_decision.reason,
                             UNIFORM_DYNAMIC_UNCONFIRMED)
            self.assertIsNone(unconfirmed_decision.trajectory)
            self.assertTrue(unconfirmed_decision.clear)
            from fwis_v3_optimizer.uniform_pipeline import _retime_unconfirmed_phases

            recovered, factors = _retime_unconfirmed_phases(unconfirmed_plan, unconfirmed)
            self.assertEqual(set(factors), {0})
            self.assertEqual(recovered.edges, unconfirmed_plan.edges)
            self.assertGreater(recovered.duration_s, unconfirmed_plan.duration_s)
            self.assertEqual(recovered.phases[0].duration_s,
                             unconfirmed_plan.phases[0].duration_s)
            rebuilt = UniformTimedPlan.from_model_json(recovered.to_model_json())
            self.assertEqual(rebuilt.model_sha256(), recovered.model_sha256())
            confirmed = audit(rebuilt, rebuilt.sample())
            self.assertTrue(confirmed.feasible, confirmed.detail)
            self.assertTrue(confirmed.metrics["dynamic_continuous_certificate"])
            passed(record, "unconfirmed_arc_is_not_publishable",
                   model_input_file=unconfirmed_file,
                   timing_limits={"jerk_max": limits.jerk_max},
                   duration_s=unconfirmed_plan.duration_s,
                   dynamic_continuous_certificate=False,
                   confirmation=confirmation,
                   rejection=failure_evidence(unconfirmed),
                   gate=unconfirmed_decision.as_status(),
                   not_a_physical_exceedance=True,
                   note="the candidate grid peak is below the jerk limit; "
                        "only the conservative full-interval bound exceeds "
                        "it, so this is UNIFORM_DYNAMIC_UNCONFIRMED rather "
                        "than FORMATION_RETIMING_INFEASIBLE")

    # -- results -----------------------------------------------------------
    @classmethod
    def tearDownClass(cls):
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        import_paths = {}
        source_hashes = {}
        for module_name in (
                "uniform_quality", "uniform_trajectory", "formation_quality",
                "formation_collision", "formation_retiming"):
            module = sys.modules.get("fwis_v3_optimizer." + module_name)
            import_paths[module_name] = (
                getattr(module, "__file__", None) if module else None)
        for name in SOURCE_FILES:
            path = PACKAGE_SRC / "fwis_v3_optimizer" / name
            source_hashes[name] = hashlib.sha256(
                path.read_bytes()).hexdigest()
        payload = {
            "schema": "fr02b.uniform_quality_results.v2",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "status": "PASSED" if RESULTS and all(
                item["status"] == "PASSED" for item in RESULTS) else "FAILED",
            "python": sys.version,
            "platform": platform.platform(),
            "repo_root": str(REPO_ROOT),
            "package_src": str(PACKAGE_SRC),
            "import_paths": import_paths,
            "source_hashes": source_hashes,
            "acceptance_thresholds": dict(ACCEPTANCE_THRESHOLDS),
            "dynamic_candidate_count": 803,
            "dynamic_candidates": {
                "count": len(_TAUS),
                "taus": [float(tau) for tau in _TAUS],
                "uniform_samples": 801,
                "extra_critical_taus": [
                    0.5, 0.5 - math.sqrt(3.0) / 6.0,
                    0.5 + math.sqrt(3.0) / 6.0],
                "quintic_supremum": {
                    "lambda_dot_over_length_per_duration": 1.875,
                    "lambda_ddot_over_length_per_duration_squared":
                        10.0 / math.sqrt(3.0),
                    "lambda_dddot_over_length_per_duration_cubed": 60.0,
                },
                "candidate_peak_meaning": (
                    "maximum over the shared S9 candidate grid; the grid "
                    "alone is not a continuous-time proof and is never "
                    "recorded as a continuous certificate"),
                "lipschitz_bound_meaning": (
                    "candidate peak + sup|dq/dt| * h / 2, where sup|dq/dt| "
                    "is the same-phase full-interval derivative bound and h "
                    "is the largest adjacent candidate time gap; valid "
                    "because the nearest candidate is at most h/2 away"),
                "adopted_bound_meaning": (
                    "the smaller of the analytic quintic supremum and the "
                    "Lipschitz-refined bound, per constraint and vehicle"),
                "confirmation_meaning": (
                    "a complete success requires both candidate endpoints "
                    "0 and 1 and adopted_bound / limit <= 1 + tolerance for "
                    "every motion phase, vehicle and dynamic constraint; an "
                    "unconfirmed interval returns UNIFORM_DYNAMIC_UNCONFIRMED "
                    "and is not publishable, without being reported as a "
                    "demonstrated physical exceedance"),
                "conservative_bound_meaning": (
                    "per-unit-time supremum from quintic derivative "
                    "suprema; it is retained as the analytic fallback when "
                    "no same-phase derivative bound is available"),
            },
            "cases": RESULTS,
        }
        RESULTS_PATH.write_text(json.dumps(
            payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
        collision_maps = {}
        for case in RESULTS:
            for subcase in case.get("subcases", ()):
                if "grid" in subcase and "obstacle_cells" in subcase:
                    collision_maps[subcase["name"]] = {
                        "grid": subcase["grid"],
                        "obstacle_cells_row_column": subcase[
                            "obstacle_cells"],
                        "first_collision": subcase.get("first_collision"),
                        "analytic_state": subcase.get("analytic_state"),
                        "endpoint_checks": subcase.get("endpoint_checks"),
                    }
        if collision_maps:
            map_path = ARTIFACT_DIR / "collision_map_b06.json"
            map_path.parent.mkdir(parents=True, exist_ok=True)
            map_path.write_text(json.dumps({
                "schema": "fr02b.collision_map_b06.v1",
                "map_signature_algorithm": "fr02b.grid_signature.v1",
                "cases": collision_maps,
            }, indent=2, sort_keys=True, allow_nan=False) + "\n")


def arc_phase_active(plan, time_s):
    for phase in plan.phases:
        if isinstance(phase, MotionTiming) and (
                phase.edge.primitive == "A_ARC") and (
                phase.t0_s <= time_s <= phase.end_time):
            return True
    return False


if __name__ == "__main__":
    unittest.main(verbosity=2)
