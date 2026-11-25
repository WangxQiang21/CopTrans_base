"""Start a single Ranger simulator and the selected body-Twist controller."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim = Path(get_package_share_directory("ranger_mini_gz"))
    control = Path(get_package_share_directory("fwis_control"))
    executable = PythonExpression([
        "'ranger_tracking_controller_cpp' if '", LaunchConfiguration("tracker_implementation"),
        "' == 'cpp' else 'ranger_tracking_controller'",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("tracker_implementation", default_value="python",
                              choices=["python", "cpp"]),
        DeclareLaunchArgument("algorithm", default_value="nonlinear", choices=["nonlinear", "pid"]),
        DeclareLaunchArgument("trajectory_topic", default_value="/plan/timed_trajectory"),
        DeclareLaunchArgument("config_file", default_value=str(control / "config" / "ranger_tracking.yaml")),
        DeclareLaunchArgument("body_twist_config", default_value=str(sim / "config" / "body_twist.yaml")),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / "launch" / "display.launch.py")),
            launch_arguments={
                **{key: LaunchConfiguration(key) for key in
                   ("gui", "rviz", "initial_x", "initial_y", "initial_yaw", "body_twist_config")},
                "robots": "1", "body_twist": "true", "managed_switching": "true",
            }.items()),
        Node(package="fwis_control", executable=executable, namespace="ranger1",
             parameters=[{"use_sim_time": True,
                          **{key: LaunchConfiguration(key)
                             for key in ("algorithm", "trajectory_topic", "config_file")}}],
             output="screen"),
    ])
