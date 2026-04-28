from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    livox_remote_relay = Node(
        package='cmdvel_to_px4',
        executable='livox_remote_relay',
        name='livox_remote_relay',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar_remote',
            'decimation_stride': 4,
            'output_hz': 2.0,
            'skip_nans': True,
            'min_range': 0.0,
            'max_range': 0.0,
        }],
    )

    return LaunchDescription([
        livox_remote_relay,
    ])
