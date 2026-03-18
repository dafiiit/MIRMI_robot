"""Common TF tree for docking stack.

Run this when you want consistent frames regardless of whether Vicon is running.
If Vicon is not running, the vicon/* frames will simply be disconnected.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # If other systems publish TF under 'world' (common), provide an alias
        # so consumers looking for 'vicon/world' can still resolve.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_vicon_world',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'world',
                '--child-frame-id', 'vicon/world',
            ],
        ),

        # Provide a conventional 'map' root.
        # Vicon bridge publishes with parent frame 'vicon/world' by default.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_vicon_world',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'map',
                '--child-frame-id', 'vicon/world',
            ],
        ),


        # Robot segment -> base_link (identity by convention here).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vicon_robot_to_base_link',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'vicon/Robot_1/Robot_1',
                '--child-frame-id', 'base_link',
            ],
        ),

        # Docking target marker segment -> tag center (offset).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vicon_target_to_dock_tag',
            arguments=[
                '--x', '-0.122', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.70710678', '--qw', '0.70710678',
                '--frame-id', 'vicon/Tag_0/Tag_0',
                '--child-frame-id', 'dock_tag',
            ],
        ),
        # Back-compat alias.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vicon_target_to_real_target',
            arguments=[
                '--x', '-0.122', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.70710678', '--qw', '0.70710678',
                '--frame-id', 'vicon/Tag_0/Tag_0',
                '--child-frame-id', 'vicon_real_target',
            ],
        ),

    ])

