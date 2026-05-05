"""Launch file for the Sensor Sweep Test Node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description='Absolute path to test_config.yaml (leave empty for default)',
    )

    sweep_node = Node(
        package='docking_test_suite',
        executable='sensor_sweep',
        name='sensor_sweep_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
        }],
    )

    return LaunchDescription([config_arg, sweep_node])
