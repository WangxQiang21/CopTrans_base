#!/usr/bin/env python3
"""FR06: three autonomous attempts per corridor width plus one bad goal.

Search receives only the frozen grid, the start pose/headings and the goal
pose, exactly like the offline FR04 baseline.  The frozen reference edges are
never imported here.  Every attempt runs in this one process with the same
successor/search configuration and the fixed 10000-expansion / 30 s budget.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))

import fr06_scene as scene  # noqa: E402
from fwis_v3_optimizer.formation import FormationPose  # noqa: E402
from fwis_v3_optimizer.uniform_pipeline import (  # noqa: E402
    UniformPlanningRequest, plan_uniform, request_fingerprint)
from fwis_v3_optimizer.uniform_search import GraphSearchConfig  # noqa: E402

ATTEMPTS_PER_WIDTH = 3
WIDTHS = (("wide", scene.WIDE_WIDTH_M), ("tight", scene.TIGHT_WIDTH_M))
NEGATIVE_GOAL = FormationPose(12.0, 10.0, math.pi / 2.0)


def _cpu_model():
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def _build_request(width, plan_id, goal=None, goal_headings=None):
    model = scene.formation_model()
    return UniformPlanningRequest(
        grid=scene.corridor_grid(width),
        model=model,
        vehicle_shapes=scene.vehicle_shapes(),
        start_qf=FormationPose(*scene.START_QF),
        start_headings={vehicle_id: 0.0 for vehicle_id in model.vehicle_ids},
        goal_qf=FormationPose(*scene.GOAL_QF) if goal is None else goal,
        goal_headings=goal_headings,
        plan_id=plan_id,
        formation_id=scene.FORMATION_ID,
        successor_config=scene.successor_config(),
        search_config=GraphSearchConfig())


def _selected_metrics(metrics):
    keys = (
        "certification", "checks", "collision_certificate",
        "collision_margin_m", "duration_s", "dynamic_peak_ratio",
        "dynamic_confirmation", "first_collision",
        "interior_counts", "max_finite_difference_error",
        "max_interior_geometry_error_m_or_rad", "max_mode_residual_rad",
        "max_normalized_mode_residual", "max_projection_error_m_or_rad",
        "max_rigidity_error_m", "max_state_error_m_or_rad",
        "max_wheel_end_error_rad", "max_wheel_start_error_rad",
        "model_sha256", "model_version", "static_collision_checks",
        "setup_evidence", "sweep_coverage",
    )
    return {key: metrics.get(key) for key in keys}


def _search_payload(result):
    if result.search is None:
        return None
    search = result.search
    payload = {
        "status": search.status,
        "reason": search.reason,
        "detail": search.detail,
        "first_failure_layer": search.first_failure_layer,
        "expansions": search.expansions,
        "candidate_count": search.candidate_count,
        "accepted_successors": search.accepted_successors,
        "duplicate_key_rejections": search.duplicate_key_rejections,
        "redundant_s_pruned": search.redundant_s_pruned,
        "stale_heap_entries": search.stale_heap_entries,
        "open_size": search.open_size,
        "wall_time_s": search.wall_time_s,
        "generation_s": search.generation_s,
        "collision_s": search.collision_s,
        "rejection_histogram": dict(search.rejection_histogram),
        "terminal_rejection_histogram": dict(
            search.terminal_rejection_histogram),
        "terminal_audits": list(search.terminal_audits),
    }
    if search.status == "SUCCESS":
        payload["path"] = search.as_dict(include_path=True)["path"]
        payload["goal_state"] = search.as_dict(include_path=False)[
            "goal_state"]
    return payload


def _run(width, plan_id, label, goal=None, goal_headings=None):
    request = _build_request(width, plan_id, goal, goal_headings)
    request_sha256, inputs = request_fingerprint(request)
    result = plan_uniform(request)
    payload = {
        "schema": scene.SCHEMA + ".attempt",
        "label": label,
        "corridor_width_m": width,
        "plan_id": plan_id,
        "request_sha256": request_sha256,
        "request_inputs": inputs,
        "status": result.status,
        "feasible": bool(result.feasible),
        "executable": bool(result.executable),
        "reason": result.reason,
        "detail": result.detail,
        "first_failure_layer": result.first_failure_layer,
        "wall_time_s": result.wall_time_s,
        "t_end_s": None if result.plan is None else result.plan.duration_s,
        "search": _search_payload(result),
        "audit": None,
        "phase_summary": None,
        "geometry": None,
        "plan_model_sha256": None,
        "plan_model_file": None,
    }
    if result.evaluation is not None:
        failure = result.evaluation.failure
        payload["audit"] = {
            "status": result.evaluation.status,
            "feasible": bool(result.evaluation.feasible),
            "reason": result.evaluation.reason,
            "detail": result.evaluation.detail,
            "checks": int(result.evaluation.checks),
            "failure": None if failure is None else failure.as_dict(),
            "metrics": _selected_metrics(result.evaluation.metrics),
        }
    if result.plan is not None:
        payload["phase_summary"] = scene.phase_summary(result.plan)
        payload["geometry"] = scene.formation_envelope_and_clearance(
            result.plan, width)
        name = "{}_plan{}.json".format(label, plan_id)
        target = HERE / "plans" / name
        target.write_text(result.plan.to_model_json(indent=2),
                          encoding="utf-8")
        payload["plan_model_sha256"] = result.plan.model_sha256()
        payload["plan_model_file"] = str(target.relative_to(ROOT))
    (HERE / "runs" / "{}.json".format(label)).write_text(
        json.dumps(payload, sort_keys=True, indent=2, allow_nan=False),
        encoding="utf-8")
    return payload


def _row(payload):
    search = payload["search"] or {}
    audit = payload["audit"] or {}
    metrics = audit.get("metrics") or {}
    phase = payload["phase_summary"] or {}
    geometry = payload["geometry"] or {}
    modes = phase.get("mode_block_counts") or {}
    return {
        "label": payload["label"],
        "width_m": payload["corridor_width_m"],
        "status": payload["status"],
        "feasible": payload["feasible"],
        "reason": payload["reason"],
        "first_failure_layer": payload["first_failure_layer"],
        "wall_time_s": round(payload["wall_time_s"], 6),
        "t_end_s": payload["t_end_s"],
        "expansions": search.get("expansions"),
        "candidate_count": search.get("candidate_count"),
        "accepted_successors": search.get("accepted_successors"),
        "collision_s": search.get("collision_s"),
        "generation_s": search.get("generation_s"),
        "checks": audit.get("checks"),
        "max_rigidity_error_m": metrics.get("max_rigidity_error_m"),
        "max_interior_geometry_error_m_or_rad": metrics.get(
            "max_interior_geometry_error_m_or_rad"),
        "plan_minimum_clearance_m": geometry.get("minimum_clearance_m"),
        "plan_envelope_width_m": geometry.get("envelope_width_m"),
        "plan_net_width_to_envelope_ratio": geometry.get(
            "net_width_to_envelope_ratio"),
        "reference_minimum_clearance_m": None,
        "reference_envelope_width_m": None,
        "reference_net_width_to_envelope_ratio": None,
        "mode_block_counts": modes,
        "setup_count": phase.get("setup_count"),
        "spin_block_count": phase.get("spin_block_count"),
        "ordinary_block_stops": phase.get("ordinary_block_stops"),
        "initial_setup_s": phase.get("initial_setup_s"),
        "inter_edge_setup_s": phase.get("inter_edge_setup_s"),
        "motion_s": phase.get("motion_s"),
        "total_duration_s": phase.get("total_duration_s"),
    }


def main():
    successor = scene.successor_config()
    search_config = GraphSearchConfig()
    config = {
        "successor_config": asdict(successor),
        "search_config": asdict(search_config),
        "collision_margin_m": successor.collision_margin_m,
        "budget": {"max_expansions": search_config.max_expansions,
                   "max_seconds": search_config.max_seconds},
    }
    config_sha = scene.json_sha256(config)
    # The frozen reference contributes only its goal headings and its own
    # clearance bookkeeping.  It is never passed into plan_uniform().
    reference = json.loads(
        (HERE / "reference" / "reference_summary.json").read_text())
    attempts = []
    for width_label, width in WIDTHS:
        for attempt in range(1, ATTEMPTS_PER_WIDTH + 1):
            label = "{}_{}m_attempt{}".format(width_label, width, attempt)
            plan_id = (6100 if width_label == "wide" else 6200) + attempt
            payload = _run(width, plan_id, label)
            attempts.append(_row(payload))
            print("{:24s} {:18s} wall={:6.2f}s T_end={} exp={} acc={} "
                  "layer={}".format(
                      label, payload["status"], payload["wall_time_s"],
                      payload["t_end_s"],
                      None if payload["search"] is None else
                      payload["search"]["expansions"],
                      None if payload["search"] is None else
                      payload["search"]["accepted_successors"],
                      payload["first_failure_layer"]))
    for row in attempts:
        width_label = "wide" if row["width_m"] == scene.WIDE_WIDTH_M \
            else "tight"
        frozen = reference["geometry"][width_label]
        row["reference_minimum_clearance_m"] = frozen["minimum_clearance_m"]
        row["reference_envelope_width_m"] = frozen["envelope_width_m"]
        row["reference_net_width_to_envelope_ratio"] = frozen[
            "net_width_to_envelope_ratio"]
    end_headings = {int(key): float(value)
                    for key, value in reference["goal_headings"].items()}
    negative = _run(scene.TIGHT_WIDTH_M, 6299, "collision_goal",
                    goal=NEGATIVE_GOAL, goal_headings=end_headings)
    print("{:24s} {:18s} reason={}".format(
        "collision_goal", negative["status"], negative["reason"]))

    summary = {
        "schema": scene.SCHEMA + ".experiments",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "cpu_model": _cpu_model(),
            "python": platform.python_version(),
        },
        "config": config,
        "config_sha256": config_sha,
        "attempts_per_width": ATTEMPTS_PER_WIDTH,
        "reference": {
            "model_sha256": reference["model_sha256"],
            "t_end_s": reference["phase_summary"]["total_duration_s"],
            "geometry": reference["geometry"],
            "audits": {key: {
                "width_m": value["width_m"],
                "feasible": value["evaluation"]["feasible"],
                "reason": value["evaluation"]["reason"],
            } for key, value in reference["audits"].items()},
        },
        "attempts": attempts,
        "collision_goal": {
            "goal_qf": [NEGATIVE_GOAL.x, NEGATIVE_GOAL.y, NEGATIVE_GOAL.psi],
            "goal_headings": end_headings,
            "status": negative["status"],
            "reason": negative["reason"],
            "first_failure_layer": negative["first_failure_layer"],
            "expected_rejection": True,
            "file": "runs/collision_goal.json",
        },
        "failure_policy": (
            "Every attempt is saved, including budget exhaustion. "
            "BUDGET_EXHAUSTED is a wall-clock search budget outcome, not a "
            "claim that the environment is physically infeasible."),
        "success_scope": (
            "Results describe this one 16 m x 16 m static corridor only; "
            "no global success rate is claimed."),
    }
    (HERE / "experiment_summary.json").write_text(
        json.dumps(summary, sort_keys=True, indent=2, allow_nan=False),
        encoding="utf-8")
    print("config_sha256", config_sha)
    print("wrote", HERE / "experiment_summary.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())