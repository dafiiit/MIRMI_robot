import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    config_path = os.path.join(
        get_package_share_directory('mirmi_apriltag'),
        'config',
        'tags.yaml'
    )

    # Arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='realsense',
        description='Type of camera to use: "realsense" (D435i) or "usb" (Generic)'
    )
    
    camera_type = LaunchConfiguration('camera_type')

    # 1a. RealSense Camera Node (Official Driver)
    realsense_node = Node(
        condition=LaunchConfigurationEquals('camera_type', 'realsense'),
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        # namespace='camera', # Removed to avoid /camera/camera/ topic structure
        parameters=[{
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_extrinsics': False,
            'enable_pointcloud': True,      # ist gut
            'align_depth.enable': False,       # Schalten wir erst an, wenn du Farbe+Tiefe übereinander brauchst (später gut)
            'enable_color': True,
            'enable_depth': True,
            'rgb_camera.profile': '1920x1080x10', # auf 10 Hz reduziert (von 30)
            'depth_module.profile': '1280x720x10', # auf 10 Hz reduziert (von 30)
            'filters': 'colorizer'            # Enable colorizer to publish colored depth maps (allows JPEG compression)
        }],
        remappings=[
            # Topics will be /camera/color/image_raw, /camera/depth/image_rect_raw, etc.
        ]
    )

    # 1b. USB Camera Node (Custom Node)
    usb_camera_node = Node(
        condition=LaunchConfigurationEquals('camera_type', 'usb'),
        package='jetson_camera',
        executable='usb_camera_node',
        name='usb_camera_node',
        parameters=[{'device_id': 1}], 
    )

    # 2. AprilTag Node

    # AprilTag for RealSense
    apriltag_node_rs = Node(
        condition=LaunchConfigurationEquals('camera_type', 'realsense'),
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_rs',
        parameters=[config_path],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'), 
            ('camera_info', '/camera/camera/color/camera_info'),
            ('detections', '/camera/detections')
        ]
    )

    # AprilTag for USB
    apriltag_node_usb = Node(
        condition=LaunchConfigurationEquals('camera_type', 'usb'),
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_usb',
        parameters=[config_path],
        remappings=[
            ('image_rect', '/usb_camera/image_raw'), 
            ('camera_info', '/usb_camera/camera_info'),
            ('detections', '/usb_camera/detections')
        ]
    )

    # 3. Visualizer Node
    
    # Visualizer for RealSense
    visualizer_node_rs = Node(
        condition=LaunchConfigurationEquals('camera_type', 'realsense'),
        package='mirmi_apriltag',
        executable='apriltag_visualizer',
        name='apriltag_visualizer_rs',
        remappings=[
            ('/camera/image_raw', '/camera/camera/color/image_raw'),
            ('/detections', '/camera/detections'),
            # Explicitly name the output overlay topic
            ('/camera/tag_detections_image/compressed', '/apriltag/overlay/compressed')
        ]
    )

    # Visualizer for USB
    visualizer_node_usb = Node(
        condition=LaunchConfigurationEquals('camera_type', 'usb'),
        package='mirmi_apriltag',
        executable='apriltag_visualizer',
        name='apriltag_visualizer_usb',
        remappings=[
            ('/camera/image_raw', '/usb_camera/image_raw'),
            ('/detections', '/usb_camera/detections'),
            ('/camera/tag_detections_image/compressed', '/usb_camera/tag_detections_image/compressed')
        ]
    )

    # 4. Depth Visualizer (Bandwidth optimization)
    depth_visualizer_node = Node(
        condition=LaunchConfigurationEquals('camera_type', 'realsense'),
        package='mirmi_apriltag',
        executable='depth_visualizer',
        name='depth_visualizer',
        parameters=[{
            'input_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'output_topic': '/camera/depth_visualization'
        }]
    )

    return LaunchDescription([
        camera_type_arg,
        realsense_node,
        usb_camera_node,
        apriltag_node_rs,
        apriltag_node_usb,
        visualizer_node_rs,
        visualizer_node_usb,
        depth_visualizer_node
    ])
