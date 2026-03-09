"""Launch file for Test C: Dynamic Approach."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_path', default_value='',
        description='Path to test_config.yaml (empty = use default)')

    condition_arg = DeclareLaunchArgument(
        'condition_label', default_value='',
        description='Optional condition label (for Test D use)')

    test_node = Node(
        package='docking_test_suite',
        executable='test_c_dynamic',
        name='test_c_dynamic',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'condition_label': LaunchConfiguration('condition_label'),
        }],
    )

    return LaunchDescription([config_arg, condition_arg, test_node])
