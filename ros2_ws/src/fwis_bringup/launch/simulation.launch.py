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
from ranger_mini_gz.scene_generator import (
    SCENARIOS, SCENE_VARIANTS, TASKS, generate_scene, installed_paths, load_scene,
)


def rviz_config(description, directory, scene, task, *, tracking_dashboard=False, show_corridor=False,
                interactive=False, assembly=False, experiment_id=""):
    vehicles = scene.spawn_poses(task)
    count = len(vehicles)
    data = yaml.safe_load((description / "rviz" / f"ranger_{count}.rviz").read_text())
    manager = data["Visualization Manager"]
    manager["Global Options"]["Fixed Frame"] = "map"
    manager["Displays"] = [display for display in manager["Displays"]
                           if display["Class"] != "rviz_default_plugins/Grid"]
    qos = {"Durability Policy": "Transient Local", "Reliability Policy": "Reliable",
           "History Policy": "Keep Last", "Depth": 1}
    manager["Displays"].insert(0, {
        "Class": "rviz_default_plugins/Map", "Name": "Planning map", "Enabled": True,
        "Topic": {"Value": "/map", **qos}, "Alpha": .80,
    })
    if task == "formation":
        manager["Displays"].append({
            "Class": "rviz_default_plugins/MarkerArray", "Name": "Cargo and centre mounts",
            "Enabled": True, "Topic": {"Value": "/scene/markers", **qos},
        })
        colors = ("36; 127; 176", "198; 103; 33", "103; 74; 153")
        paths = [(f"Vehicle {index} plan", f"/formation/path/vehicle_{index}", color)
                 for index, color in zip(scene.vehicle_ids, colors)]
        if assembly:
            paths += [(f"Vehicle {index} assembly", f"/formation/assembly_path/vehicle_{index}", color)
                      for index, color in zip(scene.vehicle_ids, colors)]
        if show_corridor:
            manager["Displays"].append({
                "Class": "rviz_default_plugins/MarkerArray", "Name": "Convex corridor guide",
                "Enabled": True, "Topic": {"Value": "/formation/convex_corridor_guide", **qos},
            })
        if interactive:
            manager["Displays"].append({
                "Class": "rviz_default_plugins/MarkerArray", "Name": "编队起终点",
                "Enabled": True, "Topic": {"Value": "/formation/interactive_endpoints", **qos},
            })
            tool_qos = {**qos, "Durability Policy": "Volatile"}
            manager.setdefault("Tools", []).extend([
                {"Class": "rviz_default_plugins/SetInitialPose", "Topic": {"Value": "/initialpose", **tool_qos}},
                {"Class": "rviz_default_plugins/SetGoal", "Topic": {"Value": "/move_base_simple/goal", **tool_qos}},
            ])
        if interactive or assembly:
            data.setdefault("Panels", []).append({"Class": "fwis_rviz_plugins/PlanningPanel", "Name": "规划与执行"})
    else:
        paths = [("Single-vehicle plan", "/plan/path", "36; 127; 176")]
    for name, topic, color in paths:
        manager["Displays"].append({
            "Class": "rviz_default_plugins/Path", "Name": name, "Enabled": True,
            "Topic": {"Value": topic, **qos}, "Color": color, "Alpha": 1.0,
            "Line Style": "Billboards", "Line Width": .055,
            "Offset": {"X": 0, "Y": 0, "Z": .39},
        })
    if tracking_dashboard:
        for index in vehicles:
            manager["Displays"].append({
                "Class": "rviz_default_plugins/Axes", "Name": f"Vehicle {index} control frame",
                "Enabled": True, "Reference Frame": f"robot{index}/base_link",
                "Length": .43, "Radius": .014,
            })
        manager["Displays"].append({
            "Class": "fwis_rviz_plugins/TrackingHUD", "Name": "Tracking dashboard",
            "Enabled": True, "Show Cargo Marker": False, "Experiment ID": experiment_id,
        })
    w, h = scene.size
    x, y, _yaw = scene.experiments[task]["start"]
    manager["Views"] = {
        "Current": {"Class": "rviz_default_plugins/TopDownOrtho", "X": w / 2, "Y": h / 2,
                    "Scale": min(650 / w, 590 / h)},
        "Saved": [{"Class": "rviz_default_plugins/Orbit", "Name": "Loading bay",
                   "Target Frame": "<Fixed Frame>", "Yaw": 3.45, "Pitch": .62, "Distance": 5.3,
                   "Focal Point": {"X": x + .5, "Y": y, "Z": .33}}],
    }
    data["Window Geometry"] = {"Width": 1190, "Height": 805}
    path = directory / "scene.rviz"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    return path


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
    experiment_id = ""
    if args["experiment_file"]:
        experiment_path = Path(args["experiment_file"])
        config = read_config(experiment_path)
        experiment_id = experiment_path.parent.name
        plant_overrides = config.get("plant_overrides", {})
        replay_plant = config.get("replay_plant")
        show_corridor = bool(config.get("planner_overrides", {}).get("convex_corridor"))
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
