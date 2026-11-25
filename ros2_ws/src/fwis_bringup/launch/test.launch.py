"""Run one scenario, planner and set of existing trackers on the simulation clock."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from fwis_bringup.control_affinity import allocate
from fwis_bringup.formation_route import PERMIT_TOPIC
from fwis_bringup.test_config import REAL_UNAVAILABLE, cargo_footprint_parameters, read_config
from ranger_mini_gz.scene_generator import clearance_summary, load_scene


def setup(context):
    config_path = Path(LaunchConfiguration("run_config").perform(context)).resolve()
    config = read_config(config_path)
    if config["mode"] != "sim":
        raise ValueError(REAL_UNAVAILABLE)
    if config.get("interactive") and not (config["gazebo_gui"] and config["rviz_gui"]):
        raise ValueError("Interactive experiments require Gazebo GUI and RViz2 GUI")
    directory = config_path.parent
    scene = load_scene(config["scenario"], config.get("scene_variant", "default"))
    task = config["scene_task"]
    native = config["planner_backend"] in ("single_hybrid_joint_cpp", "formation_hybrid_joint_cpp")
    robots = scene.vehicle_ids if task == "formation" else (1,)
    control_cpus = allocate(robots) if native else {}
    if control_cpus:
        print(f"Native tracker CPU sets: {control_cpus}", flush=True)
    for label in (("start",) if config.get("interactive") else ("start", "goal")):
        gaps = clearance_summary(scene, task, [(config[label], None)])
        for name, gap in gaps.items():
            if gap["distance_m"] <= 0:
                raise ValueError(f"{config['experiment']} {label}: {name} overlaps {gap['obstacle']}")

    resources = Path(get_package_share_directory("fwis_bringup")) / "launch"
    controller_source = Path(get_package_share_directory("fwis_control")) / "config" / "ranger_tracking.yaml"
    controller_config = directory / "controller_parameters.yaml"
    controller_data = config.get("replay_controller")
    if controller_data is None:
        controller_data = read_config(controller_source)
        controller_data["limits"].update(config.get("controller_limit_overrides", {}))
    controller_config.write_text(yaml.safe_dump(controller_data, sort_keys=False))
    planner_overrides = config.get("replay_planner", config["planner_overrides"])
    if task == "single":
        planner_overrides["quality"]["report_dir"] = str(directory / "trajectories" / "planner_reports")
    elif config["coordination"]["implementation"] in ("uniform_v02", "hybrid_joint") and "replay_planner" not in config:
        planner_overrides["cargo_footprint"] = cargo_footprint_parameters(
            scene.cargo.size, scene.cargo.offset)
    overrides = directory / "planner_parameters.yaml"
    overrides.write_text(yaml.safe_dump(planner_overrides, sort_keys=False))
    planner_args = {"backend": config["planner_backend"], "use_sim_time": "true",
                    "rviz": "false", "overrides_file": str(overrides)}
    if task == "formation":
        planner_args.update(formation_backend=config["coordination"]["implementation"], transport_ui="false",
                            planning_playback=str(config.get("planning_playback", True)).lower(),
                            runner_managed_execution="true")
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(resources / "plan.launch.py")),
        launch_arguments=planner_args.items())
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(resources / "simulation.launch.py")),
        launch_arguments={
            "scenario": scene.name, "variant": scene.variant, "task": task,
            "gui": str(config["gazebo_gui"]).lower(), "rviz": str(config["rviz_gui"]).lower(), "body_twist": "true",
            "tracking_dashboard": str(config["rviz_gui"]).lower(),
            "experiment_file": str(config_path), "scene_directory": str(directory / "scene"),
        }.items())
    runner = Node(package="fwis_bringup", executable="run_experiment", output="screen",
                  parameters=[{"use_sim_time": True, "run_config": str(config_path),
                               "controller_config": str(controller_config)}])
    started = False
    ready_output = ""

    def start_planner(event):
        nonlocal started, ready_output
        if started:
            return None
        ready_output = (ready_output + event.text.decode(errors="replace"))[-1024:]
        if "COPTRANS_SIMULATION_READY\n" in ready_output:
            started = True
            return [planner]
        return None

    actions = [
        RegisterEventHandler(OnProcessIO(target_action=runner, on_stdout=start_planner)),
        RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[
            EmitEvent(event=Shutdown(reason="experiment process finished"))])),
        simulation,
    ]
    for index in robots:
        parameters = {"use_sim_time": True, "algorithm": config["controller_algorithm"],
                      "config_file": str(controller_config)}
        if task == "formation":
            parameters.update(trajectory_topic=f"/robot{index}/trajectory", permit_topic=PERMIT_TOPIC,
                              future_start_window_s=config["coordination"]["lead_s"] + .1)
        if native:
            lead = config["coordination"]["lead_s"] if config["coordination"] else 1.25
            parameters.update(hybrid_joint=True, vehicle_id=index,
                              hybrid_plan_topic="/formation/playback_plan" if task == "formation" else "/hybrid_joint/plan",
                              permit_topic="/hybrid_joint/execution_permit",
                              future_start_window_s=lead + .1)
        tracker = {"python": "ranger_tracking_controller",
                   "cpp": "ranger_tracking_controller_cpp"}[config.get("tracking_implementation", "python")]
        prefix = f"taskset -c {','.join(map(str, control_cpus[index]))}" if index in control_cpus else None
        actions.append(Node(package="fwis_control", executable=tracker, prefix=prefix,
                            namespace=f"robot{index}", output="screen", parameters=[parameters],
                            # Keep network transmission off the 25 Hz control thread. The
                            # qualification monitor also measures when Gazebo consumes cmd_vel.
                            additional_env={"RMW_FASTRTPS_PUBLICATION_MODE":"ASYNCHRONOUS"} if native else {}))
    if config["record"] == "on":
        topics = ["/clock", "/map", "/tf", "/tf_static"]
        for index in robots:
            topics.extend(f"/robot{index}/{topic}" for topic in
                          ("cmd_vel", "odom", "motion_state", "controller_status", "ground_truth_pose"))
        if task == "formation":
            topics += ["/formation/plan_request", "/formation/playback_plan", "/formation/optimizer_status",
                       "/formation/job_state",
                       PERMIT_TOPIC, "/cargo/odom", "/cargo/constraint_error", "/cargo/controller_status"]
            if config.get("interactive"):
                topics += ["/formation/interactive_status", "/formation/uniform_progress",
                           "/formation/interactive_endpoints", "/initialpose", "/move_base_simple/goal"]
            topics.extend(f"/robot{index}/trajectory" for index in robots)
        else:
            topics += ["/plan/timed_trajectory", "/plan/status", "/plan/search_status"]
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", str(directory / "bag"), *topics], output="screen"))
    actions.append(runner)
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("run_config"), OpaqueFunction(function=setup)])
