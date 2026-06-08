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
    
    #this name has to match the robot name in xacro file
    robotXacroName='differential_drive_robot'
    
    #this is the name of our package, at the same ttime,
    #this is the name of the folder that will be used to define paths
    namePackage = 'mobile_robot'

    
    # Relative path to the xacro model. The default preserves the old
    # simulation model; set MOBILE_ROBOT_XACRO=model/robot_yahboom_rplidar_c1.xacro
    # when training/testing against the real Yahboom footprint.
    modelFileRelativePath = os.environ.get('MOBILE_ROBOT_XACRO', 'model/robot.xacro')
    world_name = LaunchConfiguration('world')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')
    robot_yaw = LaunchConfiguration('robot_yaw')
    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    verbose = LaunchConfiguration('verbose')
    packageSharePath = get_package_share_directory(namePackage)
    packageModelPath = os.path.join(packageSharePath, 'model')
    existingGazeboModelPath = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazeboModelPath = (
        packageModelPath
        if not existingGazeboModelPath
        else os.pathsep.join([packageModelPath, existingGazeboModelPath])
    )
    
    #this is the absolute path to the model
    pathModelFile = os.path.join(packageSharePath,modelFileRelativePath)
    
    #this is the selected Gazebo world file inside the package model directory
    pathWorldFile = PathJoinSubstitution([packageSharePath, 'model', world_name])
    #get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Start gzserver with GazeboRosFactory explicitly enabled.
    # /spawn_entity is only available when libgazebo_ros_factory.so is loaded.
    gazeboServerLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(server),
        launch_arguments={
            'world': pathWorldFile,
            'verbose': verbose,
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
        }.items()
    )

    gazeboClientLaunch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
        launch_arguments={
            'verbose': verbose,
        }.items()
    )
    
    #here, we create a gazebo node
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py', 
                          arguments=[
                              '-topic', 'robot_description',
                              '-entity', robotXacroName,
                              '-x', robot_x,
                              '-y', robot_y,
                              '-z', robot_z,
                              '-Y', robot_yaw,
                          ],
                          output='screen')
    
    
    #robot state publisher
    nodeRobotStatePublisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description':robotDescription,
                     'use_sim_time':True}]
    )
    
    
    #here we create an empty launch description
    launchDescriptionObject= LaunchDescription()
    
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'world',
            default_value='retrain_stage0_empty.world',
            description='Gazebo world filename from mobile_robot/model',
        )
    )
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'robot_x',
            default_value='0.0',
            description='Initial robot x position in Gazebo world frame',
        )
    )
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'robot_y',
            default_value='0.0',
            description='Initial robot y position in Gazebo world frame',
        )
    )
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'robot_z',
            default_value='0.08',
            description='Initial robot z position; 0.08 keeps the wheels near the floor',
        )
    )
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'robot_yaw',
            default_value='0.0',
            description='Initial robot yaw in radians',
        )
    )
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run Gazebo headless.',
        )
    )
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'server',
            default_value='true',
            description='Set to "false" not to run gzserver.',
        )
    )
    launchDescriptionObject.add_action(
        DeclareLaunchArgument(
            'verbose',
            default_value='true',
            description='Print verbose Gazebo server/client output.',
        )
    )
    launchDescriptionObject.add_action(LogInfo(msg=['[LAUNCH] Using world: ', world_name]))
    launchDescriptionObject.add_action(
        LogInfo(msg=[
            '[LAUNCH] Robot spawn: x=', robot_x,
            ' y=', robot_y,
            ' z=', robot_z,
            ' yaw=', robot_yaw,
        ])
    )
    launchDescriptionObject.add_action(
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazeboModelPath)
    )

    # Start Gazebo with ROS init/factory plugins so /spawn_entity exists.
    launchDescriptionObject.add_action(gazeboServerLaunch)
    launchDescriptionObject.add_action(gazeboClientLaunch)
    
    # Publish robot_description before spawn_entity waits for it.
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(
        TimerAction(
            period=3.0,
            actions=[spawnModelNode],
        )
    )
    
    return launchDescriptionObject

    
