"""Generate and show a CopTrans industrial scene with Gazebo ground truth."""

from dataclasses import replace
import os
from pathlib import Path
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from fwis_bringup.experiment_log import read_config
from ranger_mini_gz.model_launch import body_bridges, model_actions
from ranger_mini_gz.rviz_layout import rviz_config
from ranger_mini_gz.scene_generator import (
    SCENARIOS, SCENE_VARIANTS, TASKS, generate_scene, installed_paths, load_scene,
)


def setup(context):
    args = {key: LaunchConfiguration(key).perform(context) for key in
            ("scenario", "variant", "resolution", "task", "gui", "rviz", "body_twist", "tracking_dashboard",
             "experiment_file", "scene_directory")}
    paths = installed_paths()
    scene = load_scene(args["scenario"], args["variant"], args["resolution"] or None, paths)
    task = args["task"]
    body_twist = args["body_twist"] == "true"
    plant_overrides = {}
    replay_plant = None
    show_corridor = interactive = assembly = False
    show_cargo_trace = True
    experiment_id = ""
    if args["experiment_file"]:
        experiment_path = Path(args["experiment_file"])
        config = read_config(experiment_path)
        experiment_id = experiment_path.parent.name
        plant_overrides = config.get("plant_overrides", {})
        replay_plant = config.get("replay_plant")
        show_corridor = bool(config.get("planner_overrides", {}).get("convex_corridor"))
        show_cargo_trace = config.get("planner_backend") == "formation_hybrid_joint_cpp"
        interactive = bool(config.get("interactive"))
        assembly = bool(config.get("assembly"))
        experiment = {key: config[key] for key in ("start", "goal", "waypoints")}
        scene = replace(scene, experiments={**scene.experiments, task: experiment})
    actions = []
    if args["scene_directory"]:
        directory = Path(args["scene_directory"])
    else:
        temporary = TemporaryDirectory(prefix=f"coptrans-{scene.name}-")
        directory = Path(temporary.name)
        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _context: temporary.cleanup())])))
    template = paths[0] / "config" / "ranger.gui.config"
    spawns = (dict(zip(scene.vehicle_ids, scene.assembly_starts, strict=True)) if assembly else
              scene.spawn_poses(task))
    generate_scene(scene, directory, task, template, audit=False, spawn_poses=spawns)
    body_settings = None
    if body_twist:
        plant_source = paths[0] / "config" / "body_twist.yaml"
        plant_data = replay_plant
        if plant_data is None:
            plant_data = yaml.safe_load(plant_source.read_text())
            plant_data["plant"].update(plant_overrides)
        plant_config = directory / "body_twist_parameters.yaml"
        plant_config.write_text(yaml.safe_dump(plant_data, sort_keys=False))
        body_settings = str(plant_config)
    def simulator_process(label, process):
        if body_twist:
            def on_exit(_event, context):
                if not context.is_shutdown:
                    return [EmitEvent(event=Shutdown(reason=f"{label} exited during the experiment"))]
                return []
            actions.append(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=on_exit)))
        actions.append(process)

    simulator_process("Gazebo server", ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v", "2", str(directory / "world.sdf")], output="screen"))
    if args["gui"] == "true":
        simulator_process("Gazebo GUI", ExecuteProcess(
            cmd=["gz", "sim", "-g", "-v", "2", "--gui-config", str(directory / "gui.config")],
            output="screen"))
    description = Path(get_package_share_directory("ranger_mini_description"))
    topic = f"/world/{scene.name}/clock"
    bridges = [f"{topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    remaps = [(topic, "/clock")]
    trace_library = os.environ.get("COPTRANS_TRACE_RCL_TAKE_SO")
    bridge_environment = None
    if trace_library:
        preload = (trace_library, os.environ.get("LD_PRELOAD"))
        bridge_environment = {"LD_PRELOAD": os.pathsep.join(filter(None, preload))}
    for index, placement in spawns.items():
        robot = f"robot{index}"
        models, bridge, remap = model_actions(
            description, scene.name, robot, placement,
            ("white", "orange", "blue")[index - 1], "map", ground_truth=True,
            body_settings=body_settings,
            managed_switching="true" if body_twist else "false")
        actions.extend(models)
        bridges.append(bridge)
        remaps.append(remap)
        if body_twist:
            actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                                name=f"{robot}_body_bridge", arguments=body_bridges(robot),
                                additional_env=bridge_environment, output="screen"))
    actions.extend([
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="scene_bridge",
             arguments=bridges, remappings=remaps),
        Node(package="ranger_mini_gz", executable="scene_map", output="screen",
             parameters=[{"directory": str(directory), "use_sim_time": True,
                          "live_cargo": body_twist and task == "formation"}]),
    ])
    if args["rviz"] == "true":
        tracking_dashboard = args["tracking_dashboard"] == "true"
        hud_remaps = [
            (f"/ranger{index}/{topic}", f"/robot{index}/{topic}")
            for index in spawns
            for topic in ("odom", "motion_state", "controller_status")
        ] if tracking_dashboard else []
        config = rviz_config(description, directory, scene, task,
                             tracking_dashboard=tracking_dashboard, show_corridor=show_corridor,
                             show_cargo_trace=show_cargo_trace,
                             interactive=interactive, assembly=assembly, experiment_id=experiment_id)
        simulator_process("RViz2", Node(package="rviz2", executable="rviz2",
                            arguments=["-d", str(config)], remappings=hud_remaps,
                            parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=SCENARIOS[0], choices=SCENARIOS),
        DeclareLaunchArgument("variant", default_value="auto", choices=SCENE_VARIANTS),
        DeclareLaunchArgument("task", default_value="formation", choices=TASKS),
        DeclareLaunchArgument("resolution", default_value=""),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("body_twist", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("tracking_dashboard", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("experiment_file", default_value=""),
        DeclareLaunchArgument("scene_directory", default_value=""),
        OpaqueFunction(function=setup),
    ])
