"""Launch file for Test A: Static Distance Profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_path', default_value='',
        description='Path to test_config.yaml (empty = use default)')

    test_node = Node(
        package='docking_test_suite',
        executable='test_a_distance',
        name='test_a_distance',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
        }],
    )

    return LaunchDescription([config_arg, test_node])
