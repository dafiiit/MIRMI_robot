"""Common TF tree for docking stack.

Run this when you want consistent frames regardless of whether Vicon is running.
If Vicon is not running, the vicon/* frames will simply be disconnected.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rsp_launch_file = os.path.join(
        get_package_share_directory('mirmi_robot_description'),
        'launch',
        'rsp.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rsp_launch_file)
        ),
    ])
