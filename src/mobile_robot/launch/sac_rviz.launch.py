import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("mobile_robot")

    model_xacro = LaunchConfiguration("model_xacro")
    rviz_config = LaunchConfiguration("rviz_config")
    publish_robot_model = LaunchConfiguration("publish_robot_model")
    show_center_success_circle = LaunchConfiguration("show_center_success_circle")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(["xacro ", model_xacro]),
        value_type=str,
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="sac_joint_state_publisher",
        output="screen",
        condition=IfCondition(publish_robot_model),
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sac_robot_state_publisher",
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
        name="sac_rviz",
        output="screen",
        arguments=["-d", rviz_config],
    )

    center_success_marker = Node(
        package="mobile_robot",
        executable="center_success_marker_node",
        name="center_success_marker",
        output="screen",
        condition=IfCondition(show_center_success_circle),
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "frame_id": "map",
                "center_x": ParameterValue(
                    LaunchConfiguration("center_circle_x"), value_type=float
                ),
                "center_y": ParameterValue(
                    LaunchConfiguration("center_circle_y"), value_type=float
                ),
                "radius": ParameterValue(
                    LaunchConfiguration("center_circle_radius"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "publish_robot_model",
                default_value="true",
                description="Publish robot_description and zero joint states for RViz RobotModel.",
            ),
            DeclareLaunchArgument(
                "model_xacro",
                default_value=os.path.join(package_share, "model", "robot.xacro"),
                description="Robot Xacro used for RViz RobotModel display.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "sac_real_robot.rviz"]
                ),
                description=(
                    "RViz config with Map, LiDAR, odom, TF, AMCL particles, "
                    "RobotModel, and center success circle."
                ),
            ),
            DeclareLaunchArgument(
                "show_center_success_circle",
                default_value="true",
                description="Show a 0.22 m radius circle at the map center in RViz.",
            ),
            DeclareLaunchArgument(
                "center_circle_x",
                default_value="0.0",
                description="Center circle X position in the map frame.",
            ),
            DeclareLaunchArgument(
                "center_circle_y",
                default_value="0.0",
                description="Center circle Y position in the map frame.",
            ),
            DeclareLaunchArgument(
                "center_circle_radius",
                default_value="0.22",
                description="Center circle radius in meters.",
            ),
            joint_state_publisher,
            robot_state_publisher,
            center_success_marker,
            rviz,
        ]
    )
