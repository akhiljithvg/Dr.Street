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

    video_stream = Node(
        package='duckie_perception',
        executable='video_stream_node',
        name='video_stream_node',
        output='screen',
        parameters=[{
            'video_device': 0,
            'frame_width': 320,
            'frame_height': 240,
            'stream_port': 5000,
            'stream_host': '0.0.0.0',
            'fps': 30,
        }],
    )

    return LaunchDescription([
        followlane,
        video_stream,
    ])
