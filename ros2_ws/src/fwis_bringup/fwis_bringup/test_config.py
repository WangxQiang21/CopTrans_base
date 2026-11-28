"""Parse the single public test entry point without starting ROS."""

import argparse
import json
import math
import os
from pathlib import Path
import sys

import yaml

from fwis_bringup.cts_cases import cargo_reference, case_poses, dock_assignment, load_cases
from fwis_bringup.experiment_log import read_config, replay_config, retest_route
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


def planner_limits(overrides, *, formation):
    """Convert the planner parameters to the native request and envelope limits."""
    vehicle = overrides.get("vehicle", {})
    motion = overrides["timing_limits"] if formation else overrides["quality"]["limits"]
    spin_speed = motion.get("yaw_rate_max" if formation else "omega_max", .4)
    spin_accel = motion.get("yaw_accel_max" if formation else "alpha_max", .45)
    spin_jerk = motion.get("yaw_jerk_max" if formation else "angular_jerk_max", 3. if formation else 1.)
    return dict(
        radius=motion.get("r_min_ackermann", .9) if formation else vehicle.get("r_min_ack", .9),
        speed=motion.get("v_max", .16), acceleration=motion.get("a_max", .22),
        jerk=motion.get("jerk_max", 2. if formation else .8),
        lateral_acceleration=motion.get("lateral_accel_max" if formation else "lateral_a_max",
                                        .6 if formation else 2.),
        spin_speed=spin_speed, spin_acceleration=spin_accel, spin_jerk=spin_jerk,
        yaw_rate=spin_speed if formation else motion.get("translation_omega_max", .4),
        yaw_acceleration=spin_accel if formation else motion.get("translation_alpha_max", .45),
        yaw_jerk=spin_jerk if formation else motion.get("translation_angular_jerk_max", 1.),
        wheelbase=vehicle.get("wheelbase", .494), track=vehicle.get("track", .364),
        wheel_speed=motion.get("wheel_speed_max", 1. if formation else 3.),
        steer_angle=motion.get("steer_angle_max", math.pi / 2),
        steer_rate=motion.get("steer_rate_max", .8),
        steer_acceleration=motion.get("steer_accel_max", 1.5),
        min_motion_duration=motion.get("min_mode_dwell_s", .30),
        prepare_time=(motion if formation else vehicle).get("mode_switch_duration", .70),
        cargo_center_speed=motion.get("cargo_center_speed", 0.),
        high_steer_angle=motion.get("high_steer_angle", math.radians(20)),
        high_steer_speed=motion.get("high_steer_speed", 0.),
    )


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
        description="Run CopTrans in Gazebo and RViz2. Multi-vehicle runs wait for RViz endpoints by default.",
        epilog="For an automatic formation run, use ./scripts/test.sh --task multi --experiment full_route")
    parser.add_argument("--mode", choices=("sim", "real"), default="sim")
    parser.add_argument("--task", choices=("single", "multi"), default="single")
    parser.add_argument("--scenario", choices=scenarios)
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
    parser.add_argument("--case", help="Run a frozen CTS long-route case by ID")
    parser.add_argument("--dock-assignment", default="auto",
                        help="auto or the dock numbers for robot1, robot2, robot3, such as 1,3,2")
    parser.add_argument("--headless", action="store_true", help="No Gazebo or RViz GUI, fixed runs only")
    parser.add_argument("--formation", choices=(registry["formation"]["name"],), help="multi only")
    parser.add_argument("--experiment", choices=tuple(experiments),
                        help="Run a fixed route automatically unless --interactive is also set")
    parser.add_argument("--record", choices=("on", "off"), default="off",
                        help="Capture an extra topic bag. Compact local experiment logs are always saved.")
    history = parser.add_mutually_exclusive_group()
    history.add_argument("--replay", metavar="EXPERIMENT_ID", help="Re-run a local experiment with its archived settings")
    history.add_argument("--retest", metavar="EXPERIMENT_ID", help="Use archived map and selected poses with the currently selected planner and limits")
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
    try:
        args.dock_assignment = dock_assignment(args.dock_assignment)
    except ValueError as error:
        parser.error(str(error))
    if args.case and (args.task != "multi" or args.interactive or args.replay or args.retest
                      or args.experiment not in (None, "full_route")):
        parser.error("--case requires a new fixed --task multi run")
    if args.case:
        args.experiment = "full_route"
    if args.replay and args.scene_variant is not None:
        parser.error("--replay uses the recorded scene layout; --scene-variant is only for new runs")
    if args.retest and (args.interactive or args.experiment is not None or any(
            arg.split("=", 1)[0] in ("--scenario", "--scene-variant") for arg in argv)):
        parser.error("--retest already specifies the map and both selected poses")
    if args.experiment is None:
        if args.task == "multi" and not (args.replay or args.retest):
            args.interactive = True
        args.experiment = defaults["experiment"]
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
        parser.error("Interactive runs require multi, the optimized architecture and planning playback. "
                     "For an automatic run, specify --experiment full_route without --interactive.")
    if args.interactive and args.headless:
        parser.error("--headless requires a fixed experiment or CTS case")
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
        if args.headless:
            if config.get("interactive"):
                raise SystemExit("An interactive replay requires Gazebo and RViz")
            config.update(gazebo_gui=False, rviz_gui=False)
        if args.timeout is not None:
            config["timeout_s"] = args.timeout
        write_config(output, config, args.dry_run)
        return

    route = {}
    if args.retest:
        try:
            route = retest_route(Path(os.environ["COPTRANS_RUN_ROOT"]), args.retest)
        except (OSError, ValueError, KeyError) as error:
            raise SystemExit(f"Historical route: {error}") from None
        args.scenario, args.scene_variant, args.experiment = route["scenario"], route["scene_variant"], "full_route"
        if args.scenario not in scenarios:
            raise SystemExit(f"Historical scenario {args.scenario!r} is unavailable")
    selected_case = None
    if args.case:
        cases = load_cases(config_directory)
        if args.case not in cases:
            raise SystemExit(f"Unknown CTS case {args.case!r}; choose from {', '.join(cases)}")
        selected_case = cases[args.case]
        if (args.scenario not in (None, selected_case["scenario"]) or
                args.scene_variant not in (None, "auto", selected_case["scene_variant"])):
            raise SystemExit("A CTS case requires its frozen map")
        args.scenario, args.scene_variant = selected_case["scenario"], selected_case["scene_variant"]
    args.scenario = args.scenario or scenarios[0]
    scene_task = "single" if args.task == "single" else "formation"
    scene = read_config(scenario_directory / f"{args.scenario}.yaml")
    variant = args.scene_variant
    if variant in (None, "auto") and args.interactive and alias == "hybrid_joint":
        variant = registry["defaults"].get("interactive_scene_variants", {}).get(args.scenario, variant)
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
    layout = scene.get("obstacle_variants", {}).get(variant)
    assembly = (scene_task == "formation" and alias == "hybrid_joint" and
                isinstance(layout, dict) and bool(layout.get("assembly_starts")))
    if (args.case or args.dock_assignment) and not assembly:
        raise SystemExit("CTS cases and dock assignments require an assembly layout and the native planner")
    if selected_case:
        centers = case_poses(selected_case)
        cargo = read_config(scenario_directory.parent / "config/cargo_geometry.yaml")["cargo"]
        start, goal = (cargo_reference(centers[key], cargo["center_offset"]) for key in ("start", "goal"))
        waypoints = []
    mode = registry["formation"] if scene_task == "formation" else None
    overrides = registry["profiles"][args.task]
    speed_profile = registry["speed_profiles"][args.speed_profile]
    native_speed = speed_profile.get("native", {}) if alias == "hybrid_joint" else {}
    speeds = speed_profile.get(args.task, {}) | native_speed.get(args.task, {})
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
        if assembly:
            overrides["align_terminal_chassis"] = True
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
                  controller_limit_overrides=speed_profile.get("controller", {}) | native_speed.get("controller", {}),
                  plant_overrides=speed_profile.get("plant", {}) | native_speed.get("plant", {}),
                  tracking_implementation=args.tracker_implementation,
                  planning_playback=not args.without_planning_playback,
                  acceptance=registry["acceptance"], gazebo_gui=not args.headless, rviz_gui=not args.headless,
                  record=args.record,
                  timeout_s=args.timeout or float(experiment.get("wall_timeout_s", registry["defaults"]["timeout_s"])),
                  no_build=args.no_build,
                  observation_s=registry["defaults"]["observation_s"], interactive=args.interactive)
    if assembly:
        single = registry["profiles"]["single"]
        quality = single["quality"]
        single_speeds = speed_profile.get("single", {}) | native_speed.get("single", {})
        config["assembly_planner_overrides"] = dict(
            single, quality=dict(quality, limits=quality["limits"] | single_speeds),
            speed_profile=args.speed_profile, solve_budget_s=8., task_duration_cap_s=300.)
        config.update(assembly=True, dock_assignment=args.dock_assignment)
    if selected_case:
        config.update(cts_case=args.case, required_map_sha256=selected_case["map_sha256"])
        if "archived_docking" in selected_case:
            config["archived_docking"] = selected_case["archived_docking"]
    config.update(route)
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
