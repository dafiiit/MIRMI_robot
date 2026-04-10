import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'station_detection_LIDAR'
    
    # Include Livox Driver
    # Note: This assumes livox_ros_driver2 is in the ROS_PACKAGE_PATH
    # which is handled by sourcing the workspace in process_manager_node.py
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'launch_ROS2', 'msg_MID360_launch.py'
            )
        ])
    )

    detector_node = Node(
        package=pkg_name,
        executable='station_detector',
        name='station_detector',
        output='screen',
        parameters=[{
            'cloud_topic': '/livox/lidar',
            'z_min': 0.10,
            'z_max': 2.20,
            'grid_res': 0.10,
            'min_points_per_cell': 2,
            'min_cells_per_cluster': 20,
            'min_points_per_cluster': 50,
            'expected_length': 2.0,
            'expected_width': 1.5,
            'length_tolerance': 1.5,
            'width_tolerance': 1.5,
            'expected_range': 3.0,
            'range_tolerance': 2.0,
            'marker_mesh_resource': 'package://station_detection_LIDAR/meshes/station.stl'
        }]
    )

    return LaunchDescription([
        livox_driver,
        detector_node
    ])
