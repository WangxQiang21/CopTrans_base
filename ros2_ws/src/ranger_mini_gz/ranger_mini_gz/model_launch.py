"""Shared Gazebo spawn, robot description and root TF setup for Ranger models."""

from launch_ros.actions import Node
import xacro


# The lowest tire mesh vertex is 0.321942 m below base_link.
SPAWN_HEIGHT = 0.322


def body_bridges(robot):
    return [
        f"/{robot}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"/{robot}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{robot}/motion_state@std_msgs/msg/Int32[gz.msgs.Int32",
        f"/{robot}/sim_request_mode@std_msgs/msg/Int32]gz.msgs.Int32",
    ]


def model_actions(description, world, robot, placement, color, frame,
                  *, body_settings=None, managed_switching="false", ground_truth=False):
    urdf = str(description / "urdf" / "ranger_mini.urdf.xacro")
    mesh_dir = "meshes/painted" if color else "meshes"
    ros_urdf = xacro.process_file(urdf, mappings={
        "shell_color": color, "mesh_uri": f"package://ranger_mini_description/{mesh_dir}",
    }).toxml()
    mappings = {"shell_color": color, "mesh_uri": (description / mesh_dir).as_uri()}
    if body_settings:
        mappings.update(body_twist_settings=body_settings, managed_switching=managed_switching)
    gz_urdf = xacro.process_file(urdf, mappings=mappings).toxml()
    x, y, yaw = placement
    nodes = [
        Node(package="ros_gz_sim", executable="create", name=f"spawn_{robot}", output="screen",
             arguments=["-world", world, "-name", robot, "-string", gz_urdf,
                        "-x", str(x), "-y", str(y), "-z", str(SPAWN_HEIGHT), "-Y", str(yaw)]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace=robot, parameters=[{"robot_description": ros_urdf,
                                            "frame_prefix": f"{robot}/", "use_sim_time": True}]),
        Node(package="ranger_mini_gz", executable="model_pose_tf", namespace=robot,
             parameters=[{"child_frame": f"{robot}/base_link", "use_sim_time": True,
                          "parent_frame": frame, "ground_truth": ground_truth,
                          "pose_odometry": not bool(body_settings)}]),
    ]
    gz_topic = f"/model/{robot}/pose"
    return nodes, f"{gz_topic}@geometry_msgs/msg/PoseStamped[gz.msgs.Pose", (gz_topic, f"/{robot}/gz_pose")
