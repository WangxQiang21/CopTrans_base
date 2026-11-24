"""Display one or three Ranger Mini 2.0 models with Gazebo as the pose source."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ranger_mini_gz.model_launch import body_bridges, model_actions


def launch_models(context):
    count = int(LaunchConfiguration("robots").perform(context))
    if count not in (1, 3):
        raise ValueError("robots must be 1 or 3")
    independent = LaunchConfiguration("body_twist").perform(context) == "true"
    if independent and (count != 1 or LaunchConfiguration("motion_test").perform(context) == "true"):
        raise ValueError("body_twist requires one robot and exclusive control of its pose")
    description = Path(get_package_share_directory("ranger_mini_description"))
    sim = Path(get_package_share_directory("ranger_mini_gz"))
    cmd = ["gz", "sim", "-r", "-v", "3"]
    if LaunchConfiguration("gui").perform(context).lower() == "false":
        cmd.append("-s")
    else:
        gui_config = LaunchConfiguration("gui_config").perform(context)
        cmd.extend(["--gui-config", gui_config or str(sim / "config" / "ranger.gui.config")])
    actions = [ExecuteProcess(cmd=cmd + [str(sim / "worlds" / "whut.sdf")], output="screen")]
    bridge_args = ["/world/whut/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    bridge_remaps = [("/world/whut/clock", "/clock")]
    if independent:
        coordinates = [float(LaunchConfiguration(key).perform(context))
                       for key in ("initial_x", "initial_y", "initial_yaw")]
        if not all(map(math.isfinite, coordinates)):
            raise ValueError("spawn pose must be finite")
        config = LaunchConfiguration("body_twist_config").perform(context)
        body_settings = str(Path(config).resolve()) if config else str(sim / "config" / "body_twist.yaml")
        managed_switching = LaunchConfiguration("managed_switching").perform(context)
        bridge_args.extend(body_bridges("ranger1"))
    shell_colors = ("white", "orange", "blue") if count == 3 else ("white" if independent else "",)
    for index, shell_color in enumerate(shell_colors):
        robot = "ranger1" if independent else f"robot{index + 1}"
        placement = coordinates if independent else (0., index * 1.5, 0.)
        models, bridge, remap = model_actions(
            description, "whut", robot, placement, shell_color, "map" if independent else "world",
            body_settings=body_settings if independent else None,
            managed_switching=managed_switching if independent else "false",
        )
        actions.extend(models)
        bridge_args.append(bridge)
        bridge_remaps.append(remap)
    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                        name="ranger_bridge", arguments=bridge_args, remappings=bridge_remaps))
    actions.append(Node(package="ranger_mini_gz", executable="twist_motion_test",
                        namespace="ranger_sim", output="screen",
                        condition=IfCondition(LaunchConfiguration("motion_test")),
                        parameters=[{"robots": [f"robot{i + 1}" for i in range(count)],
                                     "use_sim_time": True}]))
    actions.append(Node(package="rviz2", executable="rviz2",
                        condition=IfCondition(LaunchConfiguration("rviz")),
                        arguments=["-d", str(description / "rviz" /
                                             ("ranger_tracking.rviz" if independent else f"ranger_{count}.rviz"))],
                        parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="1", choices=["1", "3"]),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("gui_config", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("motion_test", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("body_twist", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("body_twist_config", default_value=""),
        DeclareLaunchArgument("managed_switching", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        OpaqueFunction(function=launch_models),
    ])
