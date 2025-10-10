import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("darnet_igz")

    xacro_file = os.path.join(share_dir, "urdf", "darnetnew.xacro.urdf")
    robot_description = Command(["xacro ", xacro_file])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Launch Ignition Gazebo (new version)
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 empty.sdf"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot in Ignition Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "darnetnew",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "1.0",
        ],
        output="screen",
    )

    # Bridge between ROS 2 and Ignition Gazebo
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
        ],
        output="screen",
    )

    # Controller manager for ros2_control
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([share_dir, "config", "joint_controller.yaml"]),
        ],
        output="screen",
    )

    # Spawn controllers with event handlers
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # Event handlers for proper sequencing
    spawn_robot_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=spawn_robot,
        )
    )

    load_joint_state_broadcaster_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=load_joint_state_broadcaster,
        )
    )

    load_joint_trajectory_controller_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=load_joint_trajectory_controller,
        )
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            ignition_gazebo,
            controller_manager_node,
            gz_ros2_bridge,
            spawn_robot_event_handler,
            load_joint_state_broadcaster_event_handler,
            load_joint_trajectory_controller_event_handler,
        ]
    )
