from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    followlane = Node(
        package='duckie_perception',
        executable='followlaneesp_node',
        name='followlaneesp_node',
        output='screen',
        parameters=[{
            'video_device': 0,
            'serial_port': '/dev/ttyS0',
            'baudrate': 115200,
            'frame_width': 320,
            'frame_height': 240,
        }],
    )

    return LaunchDescription([
        followlane,
    ])
