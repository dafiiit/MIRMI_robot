
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

    # 1. USB Camera Node (Custom Node in jetson_camera package)
    camera_node = Node(
        package='jetson_camera',
        executable='usb_camera_node',
        name='usb_camera_node',
        parameters=[{'device_id': 1}],  # Default to 1 (USB Camera)
        remappings=[
            # Note: The node already publishes to /usb_camera/... by default 
            # so we don't strictly need remappings if the code is hardcoded,
            # but being explicit or allowing override is fine.
            # My code hardcoded '/usb_camera/image_raw', so no remapping needed here
            # unless we want to change it.
        ]
    )

    # 2. AprilTag Node (The math part)
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='usb_apriltag_node',
        parameters=[config_path],
        remappings=[
            # Subscribe to the USB camera topics
            ('image_rect', '/usb_camera/image_raw'), 
            ('camera_info', '/usb_camera/camera_info'),
            # Publish detections to a separate topic
            ('detections', '/usb_camera/detections')
        ]
    )

    # 3. Visualizer Node (The visualization part)
    visualizer_node = Node(
        package='mirmi_apriltag',
        executable='apriltag_visualizer',
        name='usb_apriltag_visualizer',
        remappings=[
            # Input from USB camera and its detections
            ('/camera/image_raw', '/usb_camera/image_raw'),
            ('/detections', '/usb_camera/detections'),
            # Output visualization to a separate topic
            ('/camera/tag_detections_image/compressed', '/usb_camera/tag_detections_image/compressed')
        ]
    )

    return LaunchDescription([
        camera_node,
        apriltag_node,
        visualizer_node
    ])
