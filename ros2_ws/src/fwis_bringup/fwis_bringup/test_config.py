"""Parse the single public test entry point without starting ROS."""

import argparse
import json
import math
import os
from pathlib import Path
import sys

import yaml

from ranger_mini_gz.scene_generator import SCENARIOS
from ranger_mini_gz.scene_geometry import local_pose


REAL_UNAVAILABLE = (
    "ERROR: real mode is reserved but not implemented.\n"
    "No hardware driver, CAN interface or motion command was started.\n"
    "Use --mode sim for current testing."
)


def read_config(path):
    return yaml.safe_load(path.read_text())


def cargo_footprint_parameters(size, offset):
    return dict(length_m=float(size[0]), width_m=float(size[1]),
                offset_x_m=float(offset[0]), offset_y_m=float(offset[1]))


def positive_seconds(raw):
    try:
        value = float(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError("timeout must be a positive number of seconds") from error
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("timeout must be a positive number of seconds")
    return value


def identifier(entries, name):
    if name in entries:
        return name, entries[name]
    for key, value in entries.items():
        if value == name:
            return key, value
    raise ValueError(f"unavailable backend {name!r}; choose from {', '.join(entries)}")


def parse_arguments(argv, scenarios, registry, experiments):
    defaults = registry["defaults"]
    parser = argparse.ArgumentParser(
        prog="scripts/test.sh", allow_abbrev=False,
        description="Run CopTrans in Gazebo and RViz2. Both GUIs run in every simulation.",
        epilog="Example: ./scripts/test.sh --task multi --formation wedge --dry-run")
    parser.add_argument("--mode", choices=("sim", "real"), default="sim")
    parser.add_argument("--task", choices=("single", "multi"), default="single")
    parser.add_argument("--scenario", choices=scenarios, default=scenarios[0])
    parser.add_argument("--planner-backend", help="Planner alias or ROS 2 backend ID")
    parser.add_argument("--controller-backend", default="auto", help="auto, pid, nonlinear_feedback")
    parser.add_argument("--formation", choices=(registry["formation"]["name"],), help="multi only")
    parser.add_argument("--experiment", choices=tuple(experiments), default=defaults["experiment"])
    parser.add_argument("--record", choices=("on", "off"), default="off")
    parser.add_argument("--timeout", type=positive_seconds, help="Override the experiment wall timeout")
    parser.add_argument("--no-build", action="store_true", help="Use the existing Colcon install")
    parser.add_argument("--dry-run", action="store_true", help="Print the resolved configuration and exit")
    query = parser.add_mutually_exclusive_group()
    query.add_argument("--list-scenarios", action="store_true")
    query.add_argument("--list-backends", action="store_true")
    if any(arg.split("=", 1)[0] in ("--gui", "--rviz") for arg in argv):
        parser.error("Gazebo GUI and RViz2 GUI are always enabled; --gui and --rviz are unavailable")
    args = parser.parse_args(argv)
    if args.mode == "real":
        parser.exit(2, REAL_UNAVAILABLE + "\n")
    if args.task == "single" and args.formation:
        parser.error("--formation applies only to --task multi")
    planners = registry["planners"][args.task]
    try:
        alias, backend = identifier(planners, args.planner_backend or defaults["planners"][args.task])
        controller, algorithm = identifier(
            registry["controllers"], defaults["controller"] if args.controller_backend == "auto"
            else args.controller_backend)
    except ValueError as error:
        parser.error(f"task={args.task}: {error}")
    return args, alias, backend, controller, algorithm


def resolve(argv, config_directory, scenario_directory, output):
    registry = read_config(config_directory / "test_defaults.yaml")
    experiments = read_config(config_directory / "experiments.yaml")
    scenarios = tuple(name for name in SCENARIOS if (scenario_directory / f"{name}.yaml").is_file())
    if not scenarios:
        raise ValueError("the scene registry contains no scenario files")
    args, alias, backend, controller, algorithm = parse_arguments(argv, scenarios, registry, experiments)
    if args.list_scenarios:
        print("\n".join(scenarios))
        return
    if args.list_backends:
        for task, planners in registry["planners"].items():
            print(f"{task} planners")
            for name, backend_id in planners.items():
                print(f"  {name} -> {backend_id}")
        print("controllers (both tasks)")
        print(f'  auto -> {registry["defaults"]["controller"]}')
        for name in registry["controllers"]:
            print(f"  {name}")
        return

    scene_task = "single" if args.task == "single" else "formation"
    task = read_config(scenario_directory / f"{args.scenario}.yaml")["experiments"][scene_task]
    start = task["start"]
    experiment = experiments[args.experiment]
    if experiment.get("use_scene_goal"):
        goal, waypoints = task["goal"], task.get("waypoints", [])
    else:
        dx, dy, dyaw = experiment["goal_offset"]
        x, y, _ = local_pose(start, (dx, dy))
        goal, waypoints = [x, y, start[2] + dyaw], []
    mode = registry["formation"] if scene_task == "formation" else None
    config = dict(mode=args.mode, task=args.task, scene_task=scene_task, scenario=args.scenario,
                  experiment=args.experiment, start=start, goal=goal, waypoints=waypoints,
                  planner=alias, planner_backend=backend, controller_backend=controller,
                  controller_algorithm=algorithm, formation=mode["name"] if mode else None,
                  coordination=mode, planner_overrides=registry["profiles"][args.task],
                  acceptance=registry["acceptance"], gazebo_gui=True, rviz_gui=True,
                  record=args.record,
                  timeout_s=args.timeout or float(experiment.get("wall_timeout_s", registry["defaults"]["timeout_s"])),
                  no_build=args.no_build,
                  observation_s=registry["defaults"]["observation_s"])
    if args.dry_run:
        print(yaml.safe_dump(config, sort_keys=False), end="")
    else:
        output.write_text(json.dumps(config, indent=2) + "\n")


if __name__ == "__main__":
    source = Path(__file__).resolve().parents[1]
    resolve(sys.argv[1:], source / "config", source.parent / "ranger_mini_gz" / "scenarios",
            Path(os.environ["COPTRANS_CONFIG_OUT"]))
