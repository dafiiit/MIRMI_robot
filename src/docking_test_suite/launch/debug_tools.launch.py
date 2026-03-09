"""Launch the diagnostics checker and robot driver CLI together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_path', default_value='',
        description='Path to test_config.yaml (empty = use default)')

    diagnostics_node = Node(
        package='docking_test_suite',
        executable='docking_diagnostics',
        name='docking_diagnostics',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
        }],
    )

    driver_cli_node = Node(
        package='docking_test_suite',
        executable='robot_driver_cli',
        name='robot_driver_cli',
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',  # Opens in a separate terminal for interactive input
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
        }],
    )

    return LaunchDescription([config_arg, diagnostics_node, driver_cli_node])
