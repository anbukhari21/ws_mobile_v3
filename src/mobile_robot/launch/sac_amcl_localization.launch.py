import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    package_share = get_package_share_directory("mobile_robot")
    route_pkg_prefix = "nav" + "2_"

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    amcl_params_file = LaunchConfiguration("amcl_params_file")
    auto_global_localization = LaunchConfiguration("auto_global_localization")
    auto_rotate = LaunchConfiguration("auto_rotate")
    rotate_speed = LaunchConfiguration("rotate_speed")
    rotate_duration = LaunchConfiguration("rotate_duration")
    localization_start_delay = LaunchConfiguration("localization_start_delay")
    localization_service_wait_timeout = LaunchConfiguration("localization_service_wait_timeout")
    publish_robot_model = LaunchConfiguration("publish_robot_model")

    xacro_path = os.path.join(package_share, "model", "robot.xacro")
    robot_description = xacro.process_file(xacro_path).toxml()

    map_server = Node(
        package=route_pkg_prefix + "map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            amcl_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "yaml_filename": map_yaml,
                "frame_id": map_frame,
            }
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    amcl = Node(
        package=route_pkg_prefix + "amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "global_frame_id": map_frame,
                "odom_frame_id": odom_frame,
                "base_frame_id": base_frame,
                "scan_topic": scan_topic,
                "set_initial_pose": False,
                "always_reset_initial_pose": False,
            }
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    lifecycle = Node(
        package=route_pkg_prefix + "lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_sac_localization",
        output="screen",
        parameters=[
            amcl_params_file,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        condition=IfCondition(publish_robot_model),
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_sac_localization",
        output="screen",
        condition=IfCondition(use_rviz),
    )

    autorotate_helper = Node(
        package="mobile_robot",
        executable="global_amcl_autorotate_node",
        name="global_amcl_autorotate_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "auto_global_localization": ParameterValue(
                    auto_global_localization,
                    value_type=bool,
                ),
                "auto_rotate": ParameterValue(auto_rotate, value_type=bool),
                "rotate_speed": ParameterValue(rotate_speed, value_type=float),
                "rotate_duration": ParameterValue(rotate_duration, value_type=float),
                "start_delay": ParameterValue(localization_start_delay, value_type=float),
                "service_wait_timeout": ParameterValue(
                    localization_service_wait_timeout,
                    value_type=float,
                ),
                "cmd_vel_topic": "/cmd_vel",
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(package_share, "maps", "arena_map.yaml"),
                description="Saved map YAML for AMCL localization.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "amcl_real_robot_params.yaml"]
                ),
                description="AMCL parameters. Defaults match the stable PF-S4 real robot AMCL setup.",
            ),
            DeclareLaunchArgument("auto_global_localization", default_value="true"),
            DeclareLaunchArgument("auto_rotate", default_value="true"),
            DeclareLaunchArgument("rotate_speed", default_value="0.20"),
            DeclareLaunchArgument("rotate_duration", default_value="31.5"),
            DeclareLaunchArgument("localization_start_delay", default_value="3.0"),
            DeclareLaunchArgument("localization_service_wait_timeout", default_value="120.0"),
            DeclareLaunchArgument(
                "publish_robot_model",
                default_value="true",
                description="Publish robot_description for RViz RobotModel from this AMCL launch.",
            ),
            LogInfo(
                msg=[
                    "[SAC AMCL] localization stack: map_server + amcl + lifecycle. ",
                    "Global AMCL autorotate helper is available through auto_global_localization/auto_rotate args.",
                ]
            ),
            robot_state_publisher,
            map_server,
            amcl,
            lifecycle,
            autorotate_helper,
            rviz,
        ]
    )
