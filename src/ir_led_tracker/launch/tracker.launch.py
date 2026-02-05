from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ir_led_tracker',
            executable='ir_tracker_node',
            name='ir_tracker_node',
            output='screen',
            parameters=[
                {'threshold': 40},
                {'target_frequency': 4.0},
                {'frequency_tolerance': 1.0},
                {'history_size': 60},
                {'debug_update_rate': 10.0}
            ]
        )
    ])
