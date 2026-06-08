import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = "mobile_robot"
    robot_name = "differential_drive_robot"
    package_share = get_package_share_directory(package_name)
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    server = LaunchConfiguration("server")
    verbose = LaunchConfiguration("verbose")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_z = LaunchConfiguration("robot_z")
    robot_yaw = LaunchConfiguration("robot_yaw")

    xacro_rel = os.environ.get("MOBILE_ROBOT_XACRO", "model/robot.xacro")
    model_path = os.path.join(package_share, xacro_rel)
    robot_description = xacro.process_file(model_path).toxml()
    world_path = PathJoinSubstitution([package_share, "model", world])
    package_model_path = os.path.join(package_share, "model")
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    gazebo_model_path = (
        package_model_path
        if not existing_model_path
        else os.pathsep.join([package_model_path, existing_model_path])
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")),
        condition=IfCondition(server),
        launch_arguments={
            "world": world_path,
            "verbose": verbose,
            "init": "true",
            "factory": "true",
            "force_system": "true",
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")),
        condition=IfCondition(gui),
        launch_arguments={"verbose": verbose}.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            robot_name,
            "-x",
            robot_x,
            "-y",
            robot_y,
            "-z",
            robot_z,
            "-Y",
            robot_yaw,
        ],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("world", default_value="center_stage0_empty.world"))
    ld.add_action(DeclareLaunchArgument("gui", default_value="false"))
    ld.add_action(DeclareLaunchArgument("server", default_value="true"))
    ld.add_action(DeclareLaunchArgument("verbose", default_value="false"))
    ld.add_action(DeclareLaunchArgument("robot_x", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("robot_y", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("robot_z", default_value="0.08"))
    ld.add_action(DeclareLaunchArgument("robot_yaw", default_value="0.0"))
    ld.add_action(SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path))
    ld.add_action(LogInfo(msg=["[PATH V2 GAZEBO] world=", world, " gui=", gui]))
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_robot]))
    return ld
