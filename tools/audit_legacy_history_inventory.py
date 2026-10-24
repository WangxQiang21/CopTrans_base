#!/usr/bin/env python3
"""Inventory explicit and implicit legacy archives without certifying old timing.

An archived strict-terminal/native-sweep result is a replay candidate. It is
not a restricted-profile execution certificate, especially when final actions
were never saved. All original artifacts remain read-only.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re

import yaml

ROOT = Path(__file__).resolve().parents[1]
LEGACY_SOURCES = {
    "fixed", "mode_master", "anchor_graph", "endpoint_replan", "upstream_sos",
    "upstream_suffix_replan", "iterated_joint", "joint_minlp",
    "joint_minlp_sat_repair", "joint_warm_start_refine", "joint_seed_fallback",
    "joint_mode_neighborhood", "time_clearance_repair", "coarse_seed", "repair",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory(root, excluded_run_roots=()):
    records, groups = [], {}
    for probe_path in sorted(root.rglob("probe_result.json")):
        if "source_snapshot" in probe_path.parts:
            continue
        if any(excluded in probe_path.parents for excluded in excluded_run_roots):
            continue
        run_dir = probe_path.parent
        params_path, case_path = run_dir/"params.yaml", run_dir/"case.json"
        if not params_path.exists() or not case_path.exists():
            continue
        probe = json.loads(probe_path.read_text())
        params = yaml.safe_load(params_path.read_text()) or {}
        case = json.loads(case_path.read_text())
        run_path = run_dir/"run.json"
        run = json.loads(run_path.read_text()) if run_path.exists() else {}
        configured = (params.get("planning") or {}).get("backend")
        quality = probe.get("quality_report") or {}
        reported = quality.get("backend")
        match = re.search(r"\bsource=([a-zA-Z0-9_]+)",
                          str(probe.get("optimizer", "")))
        source = match.group(1) if match else None
        if reported in ("legacy", "mode_path_v1", "mode_path_v2"):
            backend, attribution = reported, "archived_quality_report"
        elif configured in ("legacy", "mode_path_v1", "mode_path_v2"):
            backend, attribution = configured, "archived_explicit_config"
        elif source in LEGACY_SOURCES:
            backend, attribution = "legacy", "implicit_default_and_legacy_stage_source"
        else:
            backend, attribution = "unknown", "backend_not_attributed"
        xy, yaw = probe.get("position_error"), probe.get("heading_error")
        terminal_pass = (isinstance(xy, (float, int)) and math.isfinite(xy) and
                         isinstance(yaw, (float, int)) and math.isfinite(yaw) and
                         xy <= .02 and abs(yaw) <= math.radians(1))
        spacing = (run.get("checker") or {}).get("sampling_step_m")
        vehicle = params.get("vehicle") or {}
        margin = (params.get("collision") or {}).get("margin")
        body_matches = (vehicle.get("length") == .738 and
                        vehicle.get("width") == .5 and margin == .04)
        historical_candidate = bool(
            backend == "legacy" and terminal_pass and body_matches and
            probe.get("timed_sweep_clear") is True and spacing == .005 and
            probe.get("strictly_increasing_time") is True)
        item = {
            "run_dir": str(run_dir.relative_to(ROOT)),
            "probe_sha256": digest(probe_path),
            "params_sha256": digest(params_path),
            "backend": backend, "attribution": attribution,
            "configured_backend": configured, "reported_backend": reported,
            "legacy_stage_source": source,
            "case": case.get("case"), "map_dir": case.get("map_dir"),
            "map_scale": case.get("map_scale"),
            "start": case.get("start"), "goal": case.get("goal"),
            "map_signature": case.get("optimizer_map_signature"),
            "position_error_m": xy, "heading_error_rad": yaw,
            "strict_terminal_pass": terminal_pass,
            "archived_sweep_clear": probe.get("timed_sweep_clear"),
            "archived_sweep_spacing_m": spacing,
            "body_contract_matches": body_matches,
            "final_actions_captured": bool(quality.get("final_geometry")),
            "timed_trace_captured": bool(probe.get("timed_trace")),
            "historical_geometry_replay_candidate": historical_candidate,
            "restricted_execution_certified": False,
        }
        records.append(item)
        if historical_candidate:
            key = {k: item[k] for k in (
                "map_dir", "map_scale", "map_signature", "start", "goal")}
            signature = hashlib.sha256(json.dumps(
                key, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
            groups.setdefault(signature, dict(key, archive_runs=[]))[
                "archive_runs"].append(item["run_dir"])
    return records, groups


def loose_probe_inventory(root):
    """Keep early root-level probes visible without inventing missing contracts."""
    manifest = yaml.safe_load((ROOT/"tools/expv3_eval_cases.yaml").read_text())["cases"]
    records, groups = [], {}
    for path in sorted(root.glob("*.json")):
        try:
            probe = json.loads(path.read_text())
        except (ValueError, UnicodeError):
            continue
        if not isinstance(probe, dict) or "optimizer" not in probe or "start" not in probe:
            continue
        match = re.search(r"\bsource=([a-zA-Z0-9_]+)", str(probe["optimizer"]))
        source = match.group(1) if match else None
        xy, yaw = probe.get("position_error"), probe.get("heading_error")
        candidate = bool(source in LEGACY_SOURCES and
            isinstance(xy, (int, float)) and math.isfinite(xy) and xy <= .02 and
            isinstance(yaw, (int, float)) and math.isfinite(yaw) and
            abs(yaw) <= math.radians(1) and probe.get("timed_sweep_clear") is True and
            probe.get("strictly_increasing_time") is True)
        matched = [name for name, case in manifest.items()
                   if probe.get("start") == case["start"] and
                   probe.get("goal") == case["goal"]]
        item = {"path": str(path.relative_to(ROOT)), "sha256": digest(path),
                "legacy_stage_source": source, "start": probe.get("start"),
                "goal": probe.get("goal"), "position_error_m": xy,
                "heading_error_rad": yaw, "strict_terminal_native_sweep_candidate": candidate,
                "matching_input_manifest_cases": matched,
                "map_match_status": "pose-only join; historical raster/scale unverified",
                "strict_geometry_certified": False,
                "missing_evidence": ["archived params/body", "map signature",
                                     "sweep sampling contract", "final geometry"]}
        records.append(item)
        if candidate:
            key = json.dumps([probe.get("start"), probe.get("goal")], separators=(",", ":"))
            groups.setdefault(key, {"matching_input_manifest_cases": matched,
                                   "archive_paths": []})["archive_paths"].append(item["path"])
    return records, groups


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--exclude-run-root", action="append", type=Path, default=[],
                        help="exclude new validation runs from historical counts")
    args = parser.parse_args()
    excluded = tuple(p.resolve() for p in args.exclude_run_root)
    records, groups = inventory(ROOT/"relust", excluded)
    loose, loose_groups = loose_probe_inventory(ROOT/"relust")
    payload = {
        "schema": "r12_legacy_history_inventory_v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "probe archives with params/case plus early root-level probes; source snapshots excluded",
        "excluded_run_roots": [str(p.relative_to(ROOT)) for p in excluded],
        "strict_contract": {"xy_m": .02, "yaw_rad": math.radians(1),
                            "sweep_spacing_m": .005,
                            "length_m": .738, "width_m": .5,
                            "margin_total_dimension_m": .04},
        "archive_count": len(records),
        "implicit_legacy_count": sum(x["attribution"] ==
            "implicit_default_and_legacy_stage_source" for x in records),
        "historical_replay_candidate_count": sum(
            x["historical_geometry_replay_candidate"] for x in records),
        "historical_unique_input_count": len(groups),
        "restricted_execution_certificate_count": 0,
        "groups": groups, "records": records,
        "loose_probe_count": len(loose),
        "loose_strict_terminal_native_sweep_candidate_count": sum(
            x["strict_terminal_native_sweep_candidate"] for x in loose),
        "loose_candidate_unique_pose_pair_count": len(loose_groups),
        "loose_candidate_groups": loose_groups, "loose_records": loose,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({k: payload[k] for k in (
        "archive_count", "implicit_legacy_count",
        "historical_replay_candidate_count", "historical_unique_input_count")}))


if __name__ == "__main__":
    main()
