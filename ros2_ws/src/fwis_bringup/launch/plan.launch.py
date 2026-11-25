"""Start one CopTrans planner stack and optionally display its outputs."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from fwis_bringup.test_config import cargo_footprint_parameters

SINGLE_BACKENDS = (
    "single_multimodal_hybrid_astar",
    "single_constant_primitive",
    "single_continuous_mode_path",
    "single_state_lattice",
)


def read_config(path):
    return yaml.safe_load(path.read_text()) or {}


def make_nodes(context):
    backend = LaunchConfiguration("backend").perform(context)
    backend_share = Path(get_package_share_directory("fwis_backends"))
    selected_config = backend_share / "config" / backend
    common_share = Path(get_package_share_directory("fwis_common"))
    bringup_share = Path(get_package_share_directory("fwis_bringup"))
    config_arg = LaunchConfiguration("config_file").perform(context)
    override_arg = LaunchConfiguration("overrides_file").perform(context)
    overrides = read_config(Path(override_arg)) if override_arg else {}
    common = {
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context) == "true",
        "map_transient_local": LaunchConfiguration("map_transient_local").perform(context) == "true",
    }
    map_topic = LaunchConfiguration("map_topic").perform(context)
    remaps = [("/map", map_topic)]

    def planner_node(executable, params, extra_remaps=()):
        return Node(package="fwis_backends", executable=executable,
                    parameters=[*params, common], remappings=[*remaps, *extra_remaps], output="screen")

    if backend in SINGLE_BACKENDS:
        vehicle = read_config(common_share / "config" / "ranger_mini_2.yaml")
        if backend == "single_state_lattice":
            selected_config = backend_share / "config" / "single_continuous_mode_path"
        cfg_file = Path(config_arg) if config_arg else selected_config / "params.yaml"
        params = [vehicle, read_config(cfg_file), overrides]
        if backend == "single_state_lattice":
            lattice = {**params[1].get("lattice", {}), **overrides.get("lattice", {})}
            contest_legacy = lattice.get("contest_legacy", True)
            nodes = [planner_node("single_state_lattice_planner.py",
                                  [*params, {"lattice.contest_legacy": contest_legacy}])]
            if contest_legacy:
                namespace = "/plan/lattice_legacy"
                private = {
                    "/start_pose": "start", "/initialpose": "initialpose",
                    "/move_base_simple/goal": "goal", "/plan/path_coarse": "path",
                    "/plan/seed": "seed", "/plan/request": "request",
                    "/plan/seed_candidates": "seed_candidates", "/plan/markers": "markers",
                    "/plan/search_status": "search_status",
                }
                nodes.append(planner_node("single_continuous_mode_path_planner", params,
                                          [(topic, namespace + "/" + suffix)
                                           for topic, suffix in private.items()]))
        else:
            nodes = [planner_node(f"{backend}_{role}", params)
                     for role in ("planner", "optimizer", "playback")]
        rviz_file = "single.rviz"
    else:
        implementation = LaunchConfiguration("formation_backend").perform(context)
        interactive = LaunchConfiguration("transport_ui").perform(context) == "true"
        transport_path = LaunchConfiguration("transport_config").perform(context)
        contract = read_config(Path(transport_path) if transport_path else selected_config / "transport_formation.yaml")
        # ROS parameters support arrays of scalars. Preserve the YAML tuple order in JSON.
        contract["offsets"] = json.dumps(contract["offsets"])
        overrides = {**(read_config(Path(config_arg)) if config_arg else {}), **overrides}
        runner_managed = LaunchConfiguration("runner_managed_execution").perform(context) == "true"
        direct_execution = bool(overrides.get("execution_without_playback"))
        if direct_execution and (interactive or not runner_managed or implementation != "uniform_v02"):
            raise ValueError("direct execution requires the one-shot uniform experiment runner")
        if implementation == "legacy":
            params = [read_config(selected_config / "formation_params.yaml"), overrides]
            nodes = [planner_node("multi_4wis_legacy_planner", params),
                     planner_node("formation_optimizer_node.py", [*params, {"start_on_publication": True}])]
        else:
            planner_params = {"backend": implementation, "frame_id": "map",
                              "max_expansions": 10000, "max_seconds": 30.0,
                              "direct_goal_translation": False,
                              "heading_aligned_route": True, "heading_transition_route": True,
                              "goal_psi_tol": 0.017453292519943295,
                              "goal_heading_tol": 0.017453292519943295}
            if implementation == "uniform_v02":
                gazebo_share = Path(get_package_share_directory("ranger_mini_gz"))
                cargo = read_config(gazebo_share / "config" / "cargo_geometry.yaml")["cargo"]
                planner_params.update(direct_goal_translation=True, continuous_a_refinement=True,
                                      publication_dt=0.05,
                                      cargo_footprint=cargo_footprint_parameters(
                                          cargo["size"], cargo["center_offset"]))
            nodes = [planner_node("formation_uniform_planner_node.py", [planner_params, overrides])]
        playback = LaunchConfiguration("planning_playback").perform(context) == "true"
        if not playback and not direct_execution:
            raise ValueError("playback can only be omitted for runner-managed uniform execution")
        if playback:
            nodes.append(planner_node("formation_playback.py", [
                contract, {"backend": implementation, "transport_ui": interactive, "ranger_meshes": interactive,
                           "publish_tf": interactive, "hud_enabled": interactive,
                           "show_status_text": False, "show_cargo_reason": False},
                {"timing_limits": overrides["timing_limits"]} if "timing_limits" in overrides else {}]))
        if interactive:
            nodes.append(planner_node("formation_goal_bridge.py", [
                contract, {"admission_backend": implementation}, overrides]))
        rviz_file = "formation.rviz"
    nodes.append(Node(package="rviz2", executable="rviz2",
                      condition=IfCondition(LaunchConfiguration("rviz")),
                      arguments=["-d", str(bringup_share / "rviz" / rviz_file)],
                      parameters=[common], remappings=remaps))
    return nodes


def generate_launch_description():
    boolean = ["true", "false"]
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="single_continuous_mode_path",
                              choices=[*SINGLE_BACKENDS, "multi_4wis"]),
        DeclareLaunchArgument("formation_backend", default_value="uniform_v02",
                              choices=["virtual_rigid_a", "uniform_v02", "legacy"]),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("overrides_file", default_value=""),
        DeclareLaunchArgument("transport_config", default_value=""),
        DeclareLaunchArgument("transport_ui", default_value="true", choices=boolean),
        DeclareLaunchArgument("planning_playback", default_value="true", choices=boolean),
        DeclareLaunchArgument("runner_managed_execution", default_value="false", choices=boolean),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("map_transient_local", default_value="true", choices=boolean),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=boolean),
        DeclareLaunchArgument("rviz", default_value="true", choices=boolean),
        OpaqueFunction(function=make_nodes),
    ])
