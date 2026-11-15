#!/usr/bin/env python3
"""Freeze and FR02-audit the FR06 candidate reference sequence.

The reference is built only from FR01 factories.  Search never receives it:
``run_experiments.py`` imports the grids and request poses, not these edges.
"""
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import fr06_scene as scene  # noqa: E402
from fwis_v3_optimizer.formation_quality import FormationEvaluation  # noqa: E402
from fwis_v3_optimizer.uniform_quality import (  # noqa: E402
    UniformAuditRequest, audit_uniform_timed_plan)


def _evaluation_payload(evaluation):
    failure = evaluation.failure
    return {
        "status": evaluation.status,
        "feasible": bool(evaluation.feasible),
        "reason": evaluation.reason,
        "detail": evaluation.detail,
        "checks": int(evaluation.checks),
        "failure": None if failure is None else failure.as_dict(),
        "metrics": dict(evaluation.metrics),
    }


def _dense_projection(plan, dt=0.02):
    times = np.arange(0.0, plan.duration_s + 1e-9, dt)
    if times[-1] < plan.duration_s:
        times = np.append(times, plan.duration_s)
    return plan.sample(tuple(float(value) for value in times))


def main():
    plan = scene.reference_plan(plan_id=1)
    request = UniformAuditRequest.from_plan(plan)
    audits = {}
    for label, width in (("wide", scene.WIDE_WIDTH_M),
                         ("tight", scene.TIGHT_WIDTH_M),
                         ("exploratory", scene.EXPLORATORY_WIDTH_M)):
        grid = scene.corridor_grid(width)
        evaluation = audit_uniform_timed_plan(
            request, plan, plan.sample(), grid)
        audits[label] = {
            "width_m": width,
            "occupancy_sha256": scene.occupancy_sha256(width),
            "evaluation": _evaluation_payload(evaluation),
        }
        (HERE / "reference" / "audit_{}.json".format(label)).write_text(
            json.dumps(audits[label], sort_keys=True, indent=2,
                       allow_nan=False),
            encoding="utf-8")

    dense = _dense_projection(plan)
    (HERE / "reference" / "reference_model.json").write_text(
        plan.to_model_json(indent=2), encoding="utf-8")
    (HERE / "reference" / "reference_sample_projection.json").write_text(
        json.dumps(dense.as_dict(), sort_keys=True, allow_nan=False),
        encoding="utf-8")

    geometry = {}
    for label, width in (("wide", scene.WIDE_WIDTH_M),
                         ("tight", scene.TIGHT_WIDTH_M),
                         ("exploratory", scene.EXPLORATORY_WIDTH_M)):
        geometry[label] = scene.formation_envelope_and_clearance(plan, width)

    summary = {
        "schema": scene.SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "plan_id": plan.plan_id,
        "formation_id": plan.formation_id,
        "model_sha256": plan.model_sha256(),
        "model_json_sha256": scene.sha256_file(
            HERE / "reference" / "reference_model.json"),
        "sample_projection_sha256": scene.sha256_file(
            HERE / "reference" / "reference_sample_projection.json"),
        "sample_count": len(dense.states),
        "sample_dt_s": 0.02,
        "start_qf": list(scene.START_QF),
        "goal_qf": [float(plan.goal_qf.x), float(plan.goal_qf.y),
                    float(plan.goal_qf.psi)],
        "goal_headings": dict(plan.goal_headings),
        "phase_summary": scene.phase_summary(plan),
        "geometry": geometry,
        "audits": audits,
        "reference_frozen": (
            audits["wide"]["evaluation"]["feasible"]
            and audits["tight"]["evaluation"]["feasible"]),
        "exploratory_note": (
            "2.6 m is analysed only; it is not a search width. "
            "A reference rejection never proves the environment is "
            "mathematically infeasible."),
    }
    (HERE / "reference" / "reference_summary.json").write_text(
        json.dumps(summary, sort_keys=True, indent=2, allow_nan=False),
        encoding="utf-8")

    print("T_end {:.6f} s  model {}".format(plan.duration_s,
                                           summary["model_sha256"]))
    for label in ("wide", "tight", "exploratory"):
        evaluation = audits[label]["evaluation"]
        print("{:11s} {:.1f} m  feasible={} reason={} clearance={:.4f} m "
              "ratio={:.3f}".format(
                  label, audits[label]["width_m"], evaluation["feasible"],
                  evaluation["reason"],
                  geometry[label]["minimum_clearance_m"],
                  geometry[label]["net_width_to_envelope_ratio"]))
    if not summary["reference_frozen"]:
        print("FROZEN REFERENCE REJECTED", file=sys.stderr)
        return 1
    print("reference frozen: both wide and tight audits feasible")
    return 0


if __name__ == "__main__":
    sys.exit(main())