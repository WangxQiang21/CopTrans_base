#!/usr/bin/env python3
"""Audit captured legacy final actions and replay them through the shared backend.

Legacy native playback is never counted as a restricted execution certificate.
The shared replay disables downstream NLP to check that geometry availability
survives an unavailable optimizer under the frozen restricted profile.
"""
from dataclasses import asdict, fields
from datetime import datetime, timezone
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"tools"))
from diagnose_continuous_seed import load_run
from fwis_v3_optimizer.core import Action
from fwis_v3_optimizer.mode_master import ModeMasterConfig, ModePathMaster


def audit_run(run_dir):
    run_dir = run_dir.resolve()
    reports = []
    for path in (run_dir/"reports").glob("*.json"):
        report = json.loads(path.read_text())
        if report.get("status") == "LEGACY_NATIVE_PUBLISHED":
            reports.append((path, report))
    if len(reports) != 1:
        raise ValueError(f"expected one captured legacy final geometry: {run_dir}")
    path, saved = reports[0]
    evaluator, _ = load_run(run_dir)
    grid = evaluator.grid
    signature = {
        "sha256": hashlib.sha256(np.asarray(
            grid.occ, dtype=np.uint8).tobytes()).hexdigest(),
        "width": int(grid.width), "height": int(grid.height),
        "resolution": float(grid.res),
        "origin": [float(grid.ox), float(grid.oy)],
        "half_vehicle": [float(grid.half_l), float(grid.half_w)],
    }
    if signature != saved.get("map_signature"):
        raise ValueError("legacy_final_geometry_map_contract_mismatch")
    geometry = saved["final_geometry"]
    encoded = json.dumps(geometry["actions"], separators=(",", ":")).encode()
    if hashlib.sha256(encoded).hexdigest() != geometry["actions_sha256"]:
        raise ValueError("legacy_final_actions_hash_mismatch")
    actions = [Action(*row) for row in geometry["actions"]]
    strict = evaluator.evaluate(actions, timed=None,
                                allow_reverse=evaluator.allow_reverse)
    result = {
        "case": json.loads((run_dir/"case.json").read_text())["case"],
        "source_run": str(run_dir.relative_to(ROOT)),
        "source_final_report": str(path.relative_to(ROOT)),
        "source_final_report_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "final_actions_sha256": geometry["actions_sha256"],
        "legacy_source": saved["selected_source"],
        "final_action_count": len(actions),
        "map_signature_verified": True,
        "strict_legacy_final_geometry_audit": strict,
        "native_playback_restricted_profile_certified": False,
    }
    config = yaml.safe_load((run_dir/"params.yaml").read_text())
    valid = {f.name for f in fields(ModeMasterConfig)}
    search = {k: v for k, v in config["quality"]["search"].items() if k in valid}
    search.update(continuous_motion_v2=True, max_nlp_calls=0,
                  allow_reverse=evaluator.allow_reverse, allow_shunting=False)
    result["shared_replay_config"] = search
    try:
        winner, report = ModePathMaster(evaluator, ModeMasterConfig(**search)).solve(actions)
        timed = winner["timed"]
        result["shared_execution"] = {
            "status": winner["audit"]["status"],
            "source": winner["source"], "duration_s": float(timed.duration),
            "audit": winner["audit"], "report": report,
            "continuous_plan": asdict(winner["continuous_plan"]),
            "timed_trace": [[float(t), p.x, p.y, p.th, float(v), int(mode)]
                            for t, p, v, mode in timed.samples],
        }
    except (ValueError, RuntimeError) as exc:
        result["shared_execution"] = {
            "status": "REJECTED", "reason": str(exc).splitlines()[-1],
            "report": getattr(exc, "report", None),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    cases = [audit_run(p) for p in args.run_dirs]
    payload = {
        "schema": "r12_legacy_final_geometry_shared_replay_v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_geometry": "actual legacy selected final actions; no planner seed substitution",
        "execution_profile": "restricted/current configured limits",
        "downstream_nlp_disabled": True,
        "case_count": len(cases),
        "strict_geometry_pass_count": sum(
            x["strict_legacy_final_geometry_audit"]["geometry_status"] == "PASS"
            for x in cases),
        "shared_execution_pass_count": sum(
            x["shared_execution"]["status"] == "FEASIBLE" for x in cases),
        "cases": cases,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({k: payload[k] for k in (
        "case_count", "strict_geometry_pass_count", "shared_execution_pass_count")}))


if __name__ == "__main__":
    main()
