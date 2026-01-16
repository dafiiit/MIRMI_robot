import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config_path = os.path.join(
        get_package_share_directory('mirmi_apriltag'),
        'config',
        'tags.yaml'
    )

    # 1. Der offizielle Apriltag Node (Macht die Mathe)
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[config_path],
        # Remappings: Verbinde den Node mit deiner Kamera
        remappings=[
            ('image_rect', '/camera/image_raw'), # Wir nutzen raw (rect wäre besser wenn kalibriert)
            ('camera_info', '/camera/camera_info'),
            ('detections', '/detections')
        ]
    )

    # 2. Dein Visualizer (Macht das Bild bunt)
    visualizer_node = Node(
        package='mirmi_apriltag',
        executable='apriltag_visualizer',
        name='apriltag_visualizer',
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/detections', '/detections')
        ]
    )

    return LaunchDescription([
        apriltag_node,
        visualizer_node
    ])