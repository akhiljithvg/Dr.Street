import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'duckie_simulation'
    pkg_share = get_package_share_directory(package_name)

    # ==================== ROBOT STATE PUBLISHER ====================
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # ==================== GAZEBO SIM ====================
    default_world = os.path.join(pkg_share, 'worlds', 'lane_world.sdf')
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Path to Gazebo world file'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ==================== SPAWN ROBOT ====================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'duckie_bot',
            '-x', '-0.5',
            '-y', '1.0',
            '-z', '0.05',
        ],
        output='screen'
    )

    # ==================== GZ <-> ROS BRIDGE ====================
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    # ==================== SIM MOTOR BRIDGE ====================
    # Converts /cmd_motor (0-100 range) -> /cmd_vel (m/s, rad/s)
    sim_motor_bridge = Node(
        package='duckie_simulation',
        executable='sim_motor_bridge',
        name='sim_motor_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ==================== PERCEPTION NODE (centroid-based) ====================
    perception = Node(
        package='duckie_simulation',
        executable='sim_perception_node',
        name='sim_perception_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ==================== SAFETY WATCHDOG ====================
    safety = Node(
        package='duckie_safety',
        executable='watchdog_node',
        name='motor_watchdog',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ==================== LAUNCH ====================
    return LaunchDescription([
        world_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        sim_motor_bridge,
        perception,
        safety,
    ])
