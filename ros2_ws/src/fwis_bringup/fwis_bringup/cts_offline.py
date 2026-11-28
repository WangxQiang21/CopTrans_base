"""Run the frozen long-route suite through native planning without ROS nodes or Gazebo."""

import argparse
import gzip
from hashlib import sha256
import json
from pathlib import Path
import subprocess

from fwis_bringup.assembly_plan import encode_grid, native_call, native_executable, plan_assembly
from fwis_bringup.cts_cases import load_cases
from fwis_bringup.formation_route import fit_reference
from fwis_bringup.test_config import planner_limits
from ranger_mini_gz.scene_generator import load_scene, read_map, write_map
from ranger_mini_gz.scene_geometry import local_pose


def boundary(pose, headings, constrained):
    return dict(reference=pose, heading=headings, constrain_reference_yaw=True,
                constrain_heading=[constrained] * 3)


def formation_request(config, scene, grid, assembled):
    overrides = config["planner_overrides"]
    margin = overrides.get("collision_margin", .05)
    length, width = (size + 2 * margin for size in scene.vehicle_size[:2])
    start = assembled["reference"]
    goal = config["goal"]
    return dict(grid=encode_grid(grid), start=boundary(start, [assembled["docks"][i][2] for i in scene.vehicle_ids], True),
                goal=boundary(goal, [goal[2]] * 3, False), allow_reverse=True,
                task_duration_cap=overrides["task_duration_cap_s"],
                align_terminal_chassis=overrides.get("align_terminal_chassis", False),
                vehicles=[[*assembled["offsets"][i], length, width] for i in scene.vehicle_ids],
                cargo=[*scene.cargo.offset, *(size + 2 * margin for size in scene.cargo.size[:2])],
                limits=planner_limits(overrides, formation=True))


def save(path, data, compress=False):
    content = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    temp = path.with_name(path.name + ".partial")
    if compress:
        with gzip.open(temp, "wt", encoding="utf-8") as file:
            file.write(content)
    else:
        temp.write_text(content, encoding="utf-8")
    temp.replace(path)


def cruise_metrics(plan):
    """Count physical mode changes separately from stationary preparation and endpoint spins."""
    motions = [p for p in plan if p["kind"] == "motion"]
    mode_sequence = "".join(p["mode"] for p in motions)
    moving = [i for i, mode in enumerate(mode_sequence) if mode != "S"]
    interior = mode_sequence[moving[0] + 1:moving[-1]] if moving else ""
    return dict(mode_sequence=mode_sequence,
                switches=sum(a != b for a, b in zip(mode_sequence, mode_sequence[1:])),
                enroute_spins=interior.count("S"),
                spin_s=sum(p["duration"] for p in motions if p["mode"] == "S"),
                prepare_s=sum(p["duration"] for p in plan if p["kind"] == "prepare"))


def run_case(config, scene, grid, executable, directory, *, transport_only=False):
    directory.mkdir()
    assignment = config["dock_assignment"]
    fixed = dict(zip(scene.vehicle_ids, assignment, strict=True)) if assignment else None
    recorded = config.get("archived_docking")
    if recorded and (assignment is None or assignment == recorded["assignment"]):
        assignments = dict(zip(scene.vehicle_ids, recorded["assignment"], strict=True))
        docks = dict(zip(scene.vehicle_ids, recorded["observed_poses"], strict=True))
        offsets = {i: scene.offsets[slot - 1] for i, slot in assignments.items()}
        reference = fit_reference(tuple(offsets.values()), tuple(docks.values()))
        # The failed ROS request itself was not archived. Reconstruct its
        # reference and headings from the measured poses at handoff.
        assembled = dict(status="archived_docking", reference=reference, offsets=offsets,
                         assignment=assignments, docks=docks, source_experiment=config["cts_case"])
    elif transport_only:
        assignment = fixed or dict(zip(scene.vehicle_ids, range(1, 4), strict=True))
        offsets = {index: scene.offsets[slot - 1] for index, slot in assignment.items()}
        assembled = dict(status="not_run", reference=config["start"], offsets=offsets,
                         assignment=assignment, docks={i: local_pose(config["start"], offset) for i, offset in offsets.items()})
    else:
        center = local_pose(config["start"], scene.cargo.offset)
        assembled = plan_assembly(scene, grid, center, fixed=fixed, profile=config["speed_profile"],
                                  overrides=config["assembly_planner_overrides"], executable=executable)
    save(directory / "assembly.json.gz", assembled, compress=True)
    row = dict(case=config["cts_case"], status="failed", assembly_status=assembled["status"],
               assembly_s=assembled.get("wall_s"), dock_assignment=assembled.get("assignment"),
               assembly_duration_s=assembled.get("duration_s"))
    if assembled["status"] == "failed":
        return dict(row, reason=assembled["reason"])
    if assembled.get("tracks"):
        row["assembly_cruise"] = {i: cruise_metrics(track["plan"]) for i, track in assembled["tracks"].items()}
    request = formation_request(config, scene, grid, assembled)
    solve_budget = config["planner_overrides"]["solve_budget_s"]
    result = native_call(executable, "solve", request, solve_budget)
    audit = {}
    if result.get("plan"):
        audit = native_call(executable, "certify", request, 8., plan=result["plan"])
        row["transport_cruise"] = cruise_metrics(result["plan"])
        metrics = result["metrics"]
        row["transport_cruise"].update(a_length_m=metrics["a_length_m"], longest_a_m=metrics["longest_A_m"],
                                       oblique_distance_m=metrics["oblique_distance_m"])
    save(directory / "transport.json.gz", dict(request=request, result=result, certificate=audit), compress=True)
    passed = (result.get("status") == "CERTIFIED_POLICY_COMPLIANT"
              and result.get("actual_backend") == config["planner_backend"]
              and audit.get("executable") is True)
    return dict(row, status="passed" if passed else "failed", reason=result.get("reason"),
                transport_status=result.get("status"), transport_s=result.get("wall_s"),
                transport_duration_s=result.get("duration_s"), mode_sequence=(result.get("metrics") or {}).get("mode_sequence"),
                terminal_alignment=result.get("terminal_alignment"), certificate=audit.get("reason"))


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--root", required=True, type=Path)
    cli.add_argument("--output", type=Path, required=True)
    cli.add_argument("--case", action="append", help="run a fixed case by ID, repeatable")
    cli.add_argument("--suite", choices=("historical", "paper"), help="select the historical or two-map paper suite")
    cli.add_argument("--dock-assignment", default="auto")
    cli.add_argument("--speed-profile", choices=("standard", "baseline"), default="standard")
    cli.add_argument("--transport-only", action="store_true", help="test the native transport planner from an already docked state")
    args = cli.parse_args()
    root = args.root.resolve()
    out = args.output.resolve()
    cases = load_cases(root / "ros2_ws/src/fwis_bringup/config")
    ids = args.case or [name for name, row in cases.items() if row["suite"] == (args.suite or "historical")]
    if len(ids) != len(set(ids)) or not set(ids) <= cases.keys():
        cli.error("each case must be a unique registered CTS case ID")
    if args.suite and any(cases[name]["suite"] != args.suite for name in ids):
        cli.error("selected cases must belong to the requested suite")
    out.mkdir(parents=True, exist_ok=True)
    layouts = {}
    maps = {}
    for case_id in ids:
        case = cases[case_id]
        layouts[(case["scenario"], case["scene_variant"])] = case["map_sha256"]
    for (name, variant), expected in layouts.items():
        directory = out if len(layouts) == 1 else out / "maps" / name / variant
        directory.mkdir(parents=True, exist_ok=True)
        scene = load_scene(name, variant)
        write_map(scene, directory, "map", scene.resolution)
        checksum = sha256((directory / "map.pgm").read_bytes()).hexdigest()
        if checksum != expected:
            raise SystemExit(f"Frozen CTS map checksum mismatch for {name}/{variant}")
        maps[(name, variant)] = (scene, read_map(directory / "map.yaml"))
    executable = native_executable()
    summary = dict(code=subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip(),
                   source_sha256=subprocess.check_output(["python3", root / "scripts/source_fingerprint.py", root],
                                                         text=True).strip(),
                   map_checksums={f"{name}/{variant}": checksum for (name, variant), checksum in layouts.items()},
                   transport_only=args.transport_only, results=[])
    if len(layouts) == 1:
        summary["map_sha256"] = next(iter(layouts.values()))
    for case_id in ids:
        command = [root / "scripts/test.sh", "--task", "multi", "--case", case_id, "--dry-run",
                   "--dock-assignment", args.dock_assignment, "--speed-profile", args.speed_profile]
        import yaml
        config = yaml.safe_load(subprocess.check_output(command, text=True))
        scene, grid = maps[(config["scenario"], config["scene_variant"])]
        try:
            row = run_case(config, scene, grid, executable, out / case_id, transport_only=args.transport_only)
        except (RuntimeError, ValueError, OSError) as error:
            row = dict(case=case_id, status="failed", reason=str(error))
        row.update(scenario=scene.name, scene_variant=scene.variant, map_sha256=cases[case_id]["map_sha256"],
                   cargo_start_xy_deg=cases[case_id]["start"], cargo_goal_xy_deg=cases[case_id]["goal"])
        summary["results"].append(row)
        save(out / "summary.json", summary)
        print(json.dumps(row, ensure_ascii=False, allow_nan=False), flush=True)
    print(f"Offline results: {out / 'summary.json'}")
    return 0 if all(row["status"] == "passed" for row in summary["results"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
