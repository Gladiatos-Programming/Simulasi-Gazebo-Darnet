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
    share_dir = get_package_share_directory('darnet_description')
    rviz_config_file = os.path.join(share_dir, 'config', 'rvizuntukdarnet.rviz')

    xacro_file = os.path.join(share_dir, 'urdf', 'darnet.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    gui_arg = DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI")
    gui = LaunchConfiguration("gui")

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    ]),
    # Pakai empty world bawaan Ignition Fortress
    launch_arguments={"gz_args": ["-r -v 3 empty.sdf"]}.items(),
    condition=IfCondition(gui),
    )

    gazebo_headless = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    ]),
    launch_arguments={"gz_args": ["--headless-rendering -s -r -v 3 empty.sdf"]}.items(),
    condition=UnlessCondition(gui),
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
                   ],
        output="screen",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description", 
            "-name", "darnetnew", 
            "-allow_renaming", "true",
            "-z", "0.5",  # spawn 0.5 meter di atas ground
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_reader = Node(
        package='darnet_description',
        executable='imu_reader',
        name='imu_reader',
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        robot_state_publisher,
        gz_spawn_entity,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        imu_broadcaster_spawner,
        imu_reader,
        rviz_node,
    ])
    
