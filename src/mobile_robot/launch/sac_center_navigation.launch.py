import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("mobile_robot")
    workspace_root = os.path.join(os.path.expanduser("~"), "ws_mobile_v3")
    route_pkg_prefix = "nav" + "2_"

    use_gazebo = LaunchConfiguration("use_gazebo")
    gui = LaunchConfiguration("gui")
    use_amcl = LaunchConfiguration("use_amcl")
    rviz = LaunchConfiguration("rviz")
    run_motion = LaunchConfiguration("run_motion")
    run_logger = LaunchConfiguration("run_logger")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_yaw = LaunchConfiguration("goal_yaw")
    validation_csv = LaunchConfiguration("validation_csv")
    run_goal_relative = LaunchConfiguration("run_goal_relative")
    auto_global_localization = LaunchConfiguration("auto_global_localization")
    auto_rotate = LaunchConfiguration("auto_rotate")
    rotate_speed = LaunchConfiguration("rotate_speed")
    rotate_duration = LaunchConfiguration("rotate_duration")
    localization_start_delay = LaunchConfiguration("localization_start_delay")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "sac_gazebo.launch.py")
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            "world": world,
            "gui": gui,
            "server": "true",
            "verbose": "false",
        }.items(),
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "sac_amcl_localization.launch.py")
        ),
        condition=IfCondition(use_amcl),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "rviz": rviz,
            "auto_global_localization": auto_global_localization,
            "auto_rotate": auto_rotate,
            "rotate_speed": rotate_speed,
            "rotate_duration": rotate_duration,
            "localization_start_delay": localization_start_delay,
        }.items(),
    )

    route_server = Node(
        package=route_pkg_prefix + "planner",
        executable="planner_server",
        name="route_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    route_lifecycle = Node(
        package=route_pkg_prefix + "lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_sac_route",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"autostart": True},
            {"node_names": ["route_server"]},
        ],
    )

    center_route_requester = Node(
        package="mobile_robot",
        executable="center_route_requester_node",
        name="center_route_requester",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
                "goal_yaw": ParameterValue(goal_yaw, value_type=float),
            },
        ],
    )

    local_target = Node(
        package="mobile_robot",
        executable="local_target_node",
        name="local_target_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
            },
        ],
    )

    safety_filter = Node(
        package="mobile_robot",
        executable="sac_safety_filter_node",
        name="sac_safety_filter_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    sac_motion = Node(
        package="mobile_robot",
        executable="sac_motion_node",
        name="sac_motion_node",
        output="screen",
        condition=IfCondition(run_motion),
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "model_path": model_path,
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
            },
        ],
    )

    logger = Node(
        package="mobile_robot",
        executable="sac_validation_logger",
        name="sac_validation_logger",
        output="screen",
        condition=IfCondition(run_logger),
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "output_csv": validation_csv,
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
            },
        ],
    )

    goal_relative = Node(
        package="mobile_robot",
        executable="goal_relative_calculator",
        name="goal_relative_calculator",
        output="screen",
        condition=IfCondition(run_goal_relative),
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "goal_x": ParameterValue(goal_x, value_type=float),
                "goal_y": ParameterValue(goal_y, value_type=float),
                "goal_theta": ParameterValue(goal_yaw, value_type=float),
                "map_frame": "map",
                "base_frame": "base_footprint",
                "goal_tolerance_distance": 0.20,
                "goal_tolerance_angle": 0.35,
                "goal_reached_require_heading": False,
            }
        ],
    )

    bridge_nodes = TimerAction(
        period=6.0,
        actions=[
            route_server,
            route_lifecycle,
            center_route_requester,
            local_target,
            safety_filter,
            sac_motion,
            logger,
            goal_relative,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_amcl", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("run_motion", default_value="false"),
            DeclareLaunchArgument("run_logger", default_value="true"),
            DeclareLaunchArgument("run_goal_relative", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value="center_stage0_empty.world"),
            DeclareLaunchArgument("goal_x", default_value="0.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "validation_csv",
                default_value=os.path.join(workspace_root, "rl_results", "sac_navigation", "sac_motion_validation.csv"),
            ),
            DeclareLaunchArgument("auto_global_localization", default_value="true"),
            DeclareLaunchArgument("auto_rotate", default_value="true"),
            DeclareLaunchArgument("rotate_speed", default_value="0.20"),
            DeclareLaunchArgument("rotate_duration", default_value="31.5"),
            DeclareLaunchArgument("localization_start_delay", default_value="3.0"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution([package_share, "maps", "arena_map.yaml"]),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([package_share, "config", "sac_navigation_params.yaml"]),
            ),
            DeclareLaunchArgument(
                "model",
                default_value=os.path.join(workspace_root, "rl_models", "sac_goal_policy.zip"),
            ),
            LogInfo(
                msg=[
                    "[SAC AUTONOMY] /map + AMCL -> route generation -> SAC raw cmd -> emergency-only filter -> /cmd_vel. ",
                    "Runtime stack uses the saved map, live scan data, SAC motion, and emergency filtering.",
                ]
            ),
            gazebo_launch,
            amcl_launch,
            bridge_nodes,
        ]
    )
