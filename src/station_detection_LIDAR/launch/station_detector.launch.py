import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'station_detection_LIDAR'
    
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
        detector_node
    ])
