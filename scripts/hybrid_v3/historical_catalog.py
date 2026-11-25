#!/usr/bin/env python3
"""Summarize operator-selected historical routes from verified native/ROS evidence."""
import argparse
from collections import Counter, defaultdict
from hashlib import sha256
import json
import math
from pathlib import Path

from cases import ROOT, digest
from historical_routes import save


def read(path):
    return json.loads(path.read_bytes())


def ros1_lineage(folder):
    case = read(folder / "map05_uturn_long.json")
    graphs = {port: read(folder / f"{port}_historical_graph.json") for port in ("ros1", "ros2")}
    sample = graphs["ros1"]
    for key in ("map_signature", "track_sha256", "rigid_sample_count", "duration_s", "xy_sampled_length_m"):
        if sample[key] != graphs["ros2"][key]:
            raise ValueError(f"legacy ROS graph records disagree on {key}")
    if sample["map_signature"] != case["map"]["grid_sha256"]:
        raise ValueError("ROS1 graph was recorded on a different map")
    return {
        "kind": "historical_planner_publication_graph",
        "case_id": case["case_id"],
        "map": case["map"],
        "start": case["request"]["formation_start"],
        "goal": case["request"]["formation_goal"],
        "original_planner": case["backend"],
        "guidance": sample["guidance"],
        "corridor_scope": case["planner"]["virtual_a_formation_corridor_scope"],
        "goal_policy": case["planner"]["virtual_a_goal_policy"],
        "track_sha256": sample["track_sha256"],
        "sample_count": sample["rigid_sample_count"],
        "sampled_reference_length_m": sample["xy_sampled_length_m"],
        "duration_s": sample["duration_s"],
        "planning_publication_wall_s": {port: data["wall_s"] for port, data in graphs.items()},
        "source_artifact_sha256": {
            path.name: sha256(path.read_bytes()).hexdigest()
            for path in [folder / "map05_uturn_long.json", *(folder / f"{p}_historical_graph.json" for p in graphs)]
        },
    }


def summarize(summary_path, legacy_folder=None):
    summary = read(summary_path)
    if not summary["complete"] or summary["unique_requests"] != len(summary["cases"]):
        raise ValueError("a complete deduplicated native result set is required")
    maps, routes = {}, {}
    counts = defaultdict(Counter)
    for row in sorted(summary["cases"], key=lambda r: (r["archive_id"], r["task"], r["profile"])):
        proof = row["proof"]
        key = row["archive_id"]
        base = summary_path.parent / row["archive_stem"]
        req = read(base.with_suffix(".request.json"))["request"]
        contract = read(base.with_suffix(".ros_contract.json"))
        result = read(base.with_suffix(".result.json"))
        audit = read(base.with_suffix(".audit.json"))
        if (row["request_sha256"] != digest(req) or row["ros_contract_sha256"] != digest(contract) or
                row["audit"] != audit or row["status"] != result["status"]):
            raise ValueError(f"evidence changed for {base}")
        if row["passed"] != bool(result.get("plan") and result.get("actual_backend")==row["expected_backend"] and
                                 row["status"]=="CERTIFIED_POLICY_COMPLIANT" and audit.get("executable") is True):
            raise ValueError(f"incorrect pass state for {base}")
        map_sha = proof["map_sha256"]
        maps.setdefault(map_sha, {
            "scenario": proof["scenario"],
            "variant": proof["variant"],
            "resolution_m": proof["map_resolution"],
            "origin": proof["map_origin"],
            "ros_grid_sha256": row["ros_grid_sha256"],
            "obstacle_count": len(proof["obstacle_ids"]),
        })
        source = proof["source_plan"]
        if key not in routes:
            routes[key] = {
                "source_map_pgm_sha256": map_sha,
                "source_scene_manifest_sha256": proof["archive_scene_meta_sha256"],
                "start": row["start"], "goal": row["goal"],
                "euclidean_displacement_m": math.dist(row["goal"][:2],row["start"][:2]),
                "source": {
                    "task": proof["source_task"], "profile": proof["source_profile"],
                    "status": proof["source_status"], "reason": proof["source_reason"],
                    "backend": proof["source_backend"], "commit": proof["source_commit"],
                    "plan_solver_to_result_wall_s": proof["source_planner_wall_s"],
                    "plan_duration_s": proof["source_plan_duration_s"],
                    "sampled_reference_length_m": source["sampled_reference_length_m"],
                    "sample_count": source["sample_count"], "cdr_sha256": source.get("cdr_sha256"),
                    "measured_mount_error_max_m": source["max_measured_mount_error_m"],
                    "sampled_mode_switches_by_vehicle": source["sampled_switches_by_vehicle"],
                    "timing_limits": proof["old_timing_limits"],
                    "source_sdf_available": proof["stored_world_sdf_available"],
                }, "native_results": {},
            }
        if map_sha != routes[key]["source_map_pgm_sha256"]:
            raise ValueError("different maps in the same historical fixture")
        group = row["task"]+"_"+row["profile"]
        modes = row.get("modes")
        routes[key]["native_results"][group] = {
            "status": row["status"], "reason": row["reason"], "passed": row["passed"],
            "request_sha256": row["request_sha256"], "ros_contract_sha256": row["ros_contract_sha256"],
            "plan_sha256": row["plan_sha256"], "modes": modes,
            "certified_mode_switches": len(modes)-1 if row["passed"] and modes else None,
            "reference_length_m": row["reference_length_m"], "plan_duration_s": row["duration_s"],
            "solve_wall_s": row["solve_wall_s"], "independent_audit": audit.get("reason"),
            "certified_executable": audit.get("executable"),
            "source_task_profile_run": proof["same_task"] and proof["same_profile"],
            "source_timing_limits_match": proof["old_timing_matches"],
        }
        counts[group]["count"] += 1
        counts[group]["passed"] += int(row["passed"])
    catalog = dict(
        schema="coptrans.historical_routes/v1",
        experiment="Frozen native blind endpoint-only solve on the byte-verified historical map, then a separate certification process.",
        local_raw_dir=str(summary_path.parent.relative_to(ROOT)) if summary_path.parent.is_relative_to(ROOT) else str(summary_path.parent),
        source_commit=summary["source_commit"], core_commit=summary["core_commit"],
        binary_sha256=summary["binary_sha256"], library_sha256=summary["library_sha256"], policy_sha256=summary["policy_sha256"],
        raw_summary_sha256=sha256(summary_path.read_bytes()).hexdigest(), unique_routes=len(routes),
        unique_requests=len(summary["cases"]), duplicates=summary["duplicate_requests"], counts=counts, maps=maps, routes=routes,
    )
    if legacy_folder:
        catalog["separate_ros1_map05_reference"] = ros1_lineage(legacy_folder)
    return catalog


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary",type=Path)
    parser.add_argument("--legacy-folder",type=Path,help="optional location of the separate map05 ROS1/ROS2 original diagnostics")
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    save(args.output,summarize(args.summary.resolve(),args.legacy_folder))


if __name__=="__main__":
    main()
