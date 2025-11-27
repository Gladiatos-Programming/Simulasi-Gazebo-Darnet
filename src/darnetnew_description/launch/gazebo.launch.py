from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    share_dir = get_package_share_directory('darnetnew_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'darnetnew.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    gui_arg = DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI")
    gui = LaunchConfiguration("gui")

    # Launch Gazebo Classic dengan GUI
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ]),
        launch_arguments={
            "verbose": "true",
            "pause": "false",
        }.items(),
        condition=IfCondition(gui),
    )

    # Launch Gazebo Classic headless (tanpa GUI)
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
        ]),
        launch_arguments={
            "verbose": "true",
            "pause": "true",
        }.items(),
        condition=UnlessCondition(gui),
    )

    # Spawn robot di Gazebo Classic
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "darnetnew",
            "-z", "0.5",  # spawn 0.5 meter di atas ground
            "-R", "-1.047",  # roll rotation
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_urdf}],
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        # gazebo_headless,
        robot_state_publisher,
        spawn_entity,
        load_joint_trajectory_controller,
        load_joint_state_broadcaster,
    ])