"""Parse the single public test entry point without starting ROS."""

import argparse
import json
import math
import os
from pathlib import Path
import sys

import yaml

from fwis_bringup.experiment_log import read_config, replay_config
from ranger_mini_gz.scene_generator import SCENARIOS, SCENE_VARIANTS
from ranger_mini_gz.scene_geometry import local_pose


REAL_UNAVAILABLE = (
    "ERROR: real mode is reserved but not implemented.\n"
    "No hardware driver, CAN interface or motion command was started.\n"
    "Use --mode sim for current testing."
)


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
    parser.add_argument("--scene-variant", choices=SCENE_VARIANTS,
                        help="Select a named layout of the scenario for a new experiment")
    parser.add_argument("--planner-backend", help="Planner alias or ROS 2 backend ID")
    parser.add_argument("--controller-backend", default="auto", help="auto, pid, nonlinear_feedback")
    parser.add_argument("--tracker-implementation", choices=("python", "cpp"),
                        default=defaults.get("tracking_implementation", "python"))
    parser.add_argument("--architecture-profile", choices=("baseline", "optimized"),
                        default=defaults.get("architecture_profile", "baseline"))
    parser.add_argument("--speed-profile", choices=tuple(registry["speed_profiles"]),
                        default=defaults["speed_profile"])
    parser.add_argument("--without-planning-playback", action="store_true",
                        help="Omit the formation playback node in the optimized experiment")
    parser.add_argument("--multi-native-frontier", action=argparse.BooleanOptionalAction)
    parser.add_argument("--multi-convex-corridor", action=argparse.BooleanOptionalAction)
    parser.add_argument("--multi-reachable-clock", action=argparse.BooleanOptionalAction)
    parser.add_argument("--interactive", action="store_true",
                        help="Choose the formation start and goal in RViz before running the closed loop")
    parser.add_argument("--formation", choices=(registry["formation"]["name"],), help="multi only")
    parser.add_argument("--experiment", choices=tuple(experiments), default=defaults["experiment"])
    parser.add_argument("--record", choices=("on", "off"), default="off",
                        help="Capture an extra topic bag. Compact local experiment logs are always saved.")
    parser.add_argument("--replay", metavar="EXPERIMENT_ID", help="Re-run a local experiment with its archived inputs")
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
    if args.replay and args.scene_variant is not None:
        parser.error("--replay uses the recorded scene layout; --scene-variant is only for new runs")
    if args.task == "single" and args.formation:
        parser.error("--formation applies only to --task multi")
    if args.without_planning_playback and (
            args.task != "multi" or args.architecture_profile != "optimized"):
        parser.error("--without-planning-playback requires multi and --architecture-profile optimized")
    if args.task != "multi" and any(flag is not None for flag in (
            args.multi_native_frontier, args.multi_convex_corridor, args.multi_reachable_clock)):
        parser.error("multi planner options require --task multi")
    if args.interactive and (args.task != "multi" or args.architecture_profile != "optimized"
                             or args.without_planning_playback):
        parser.error("--interactive requires multi, the optimized architecture and planning playback")
    planners = registry["planners"][args.task]
    planner_default = defaults["planners"][args.task]
    if args.architecture_profile == "baseline":
        planner_default = defaults.get("baseline_planners", {}).get(args.task, planner_default)
    try:
        alias, backend = identifier(planners, args.planner_backend or planner_default)
        controller, algorithm = identifier(
            registry["controllers"], defaults["controller"] if args.controller_backend == "auto"
            else args.controller_backend)
    except ValueError as error:
        parser.error(f"task={args.task}: {error}")
    if alias == "hybrid_joint" and (args.tracker_implementation != "cpp" or args.without_planning_playback
                                   or any(flag is not None for flag in (
                                       args.multi_native_frontier, args.multi_convex_corridor,
                                       args.multi_reachable_clock))):
        parser.error("hybrid_joint uses the native tracker and its own joint solver configuration")
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

    if args.replay:
        root = Path(os.environ["COPTRANS_RUN_ROOT"])
        try:
            config = replay_config(root, args.replay)
        except (OSError, ValueError) as error:
            raise SystemExit(f"Replay: {error}") from None
        config["no_build"] = args.no_build
        if args.timeout is not None:
            config["timeout_s"] = args.timeout
        write_config(output, config, args.dry_run)
        return

    scene_task = "single" if args.task == "single" else "formation"
    scene = read_config(scenario_directory / f"{args.scenario}.yaml")
    variant = args.scene_variant
    if variant in (None, "auto"):
        variant = scene.get("default_variant", "default")
    layouts = {"default", *scene.get("obstacle_variants", {})}
    if scene.get("pinch"):
        layouts.update(scene.get("width_cases", {}))
    if variant not in layouts:
        raise SystemExit(f"Scene variant {variant!r} is not available for {args.scenario}; "
                         f"choose from {', '.join(sorted(layouts))}")
    task = scene["experiments"][scene_task]
    start = task["start"]
    experiment = experiments[args.experiment]
    if experiment.get("use_scene_goal"):
        goal, waypoints = task["goal"], task.get("waypoints", [])
    else:
        dx, dy, dyaw = experiment["goal_offset"]
        x, y, _ = local_pose(start, (dx, dy))
        goal, waypoints = [x, y, start[2] + dyaw], []
    mode = registry["formation"] if scene_task == "formation" else None
    overrides = registry["profiles"][args.task]
    speed_profile = registry["speed_profiles"][args.speed_profile]
    speeds = speed_profile.get(args.task, {})
    if scene_task == "single":
        quality = overrides["quality"]
        overrides = dict(overrides, quality=dict(quality, limits=quality["limits"] | speeds))
    else:
        overrides = dict(overrides, timing_limits=overrides["timing_limits"] | speeds)
    if alias == "hybrid_joint":
        if mode:
            mode = dict(mode, implementation="hybrid_joint")
        cap = experiment.get("task_duration_cap_s", 300.0)
        if experiment.get("use_scene_goal") and not args.interactive:
            cap = task.get("task_duration_cap_by_speed_s", {}).get(args.speed_profile, cap)
        overrides = dict(overrides, speed_profile=args.speed_profile, solve_budget_s=8.0 if scene_task == "single" else 12.0,
                         task_duration_cap_s=positive_seconds(cap))
    elif args.architecture_profile == "optimized":
        if scene_task == "formation":
            overrides = dict(overrides, reuse_terminal_certificate=True,
                             background_refinement_budget_s=1.0,
                             execution_without_playback=True)
        elif backend in ("single_continuous_mode_path", "single_state_lattice"):
            overrides = dict(overrides, search={"adaptive_reverse_competition": True},
                             quality=dict(overrides["quality"], seed_proposal_wait_s=0.0))
    if scene_task == "formation" and alias != "hybrid_joint":
        features = (registry.get("optimized_multi_defaults", {})
                    if args.architecture_profile == "optimized" else {})
        flags = {"native_frontier": args.multi_native_frontier,
                 "convex_corridor": args.multi_convex_corridor,
                 "reachability_retime": args.multi_reachable_clock}
        overrides.update(features)
        overrides.update({name: value for name, value in flags.items() if value is not None})
        if overrides.get("reachability_retime"):
            overrides["continuous_a_refinement"] = False
    if args.interactive and alias != "hybrid_joint":
        overrides = dict(overrides, publish_uniform_progress=True)
    config = dict(mode=args.mode, task=args.task, scene_task=scene_task, scenario=args.scenario,
                  scene_variant=variant,
                  experiment="interactive" if args.interactive else args.experiment,
                  start=start, goal=goal, waypoints=waypoints,
                  planner=alias, planner_backend=backend, controller_backend=controller,
                  controller_algorithm=algorithm, formation=mode["name"] if mode else None,
                  coordination=mode, planner_overrides=overrides,
                  architecture_profile=args.architecture_profile,
                  speed_profile=args.speed_profile,
                  controller_limit_overrides=speed_profile.get("controller", {}),
                  plant_overrides=speed_profile.get("plant", {}),
                  tracking_implementation=args.tracker_implementation,
                  planning_playback=not args.without_planning_playback,
                  acceptance=registry["acceptance"], gazebo_gui=True, rviz_gui=True,
                  record=args.record,
                  timeout_s=args.timeout or float(experiment.get("wall_timeout_s", registry["defaults"]["timeout_s"])),
                  no_build=args.no_build,
                  observation_s=registry["defaults"]["observation_s"], interactive=args.interactive)
    write_config(output, config, args.dry_run)


def write_config(output, config, dry_run):
    if dry_run:
        print(yaml.safe_dump(config, sort_keys=False), end="")
    else:
        output.write_text(json.dumps(config, indent=2) + "\n")


if __name__ == "__main__":
    source = Path(__file__).resolve().parents[1]
    resolve(sys.argv[1:], source / "config", source.parent / "ranger_mini_gz" / "scenarios",
            Path(os.environ["COPTRANS_CONFIG_OUT"]))
