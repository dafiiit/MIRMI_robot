import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    isaac_params_path = os.path.join(
        get_package_share_directory('station_detection_APRILTAG'),
        'config',
        'isaac_ros_apriltag_params.yaml'
    )

    # Arguments
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='realsense',
        description='Type of camera to use: "realsense" (D435i) or "usb" (Generic)'
    )

    camera_type = LaunchConfiguration('camera_type')

    # ── 1a. RealSense Camera Node (Official Driver) ───────────────────────────
    realsense_node = Node(
        condition=LaunchConfigurationEquals('camera_type', 'realsense'),
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        parameters=[{
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_extrinsics': False,
            'align_depth.enable': False,
            'pointcloud.enable': False,
            'enable_color': True,
            'enable_depth': False,
            # 1920x1080 @ 15fps was ~75% CPU. 848x480 @ 6fps was prior setting.
            # 1280x720 @ 6fps gives 2.25x more pixels for better tag detection
            # at longer range, while keeping fps low to limit CPU/DDS load.
            'rgb_camera.color_profile': '1280x720x6',
            'rgb_camera.enable_auto_exposure': True,
            # Only publish the raw transport for color/image_raw.
            # Drops compressed, compressedDepth and theora sub-topics.
            'camera.color.image_raw.enable_pub_plugins': ['image_transport/raw'],
            'publish_tf': False,
            'enable_sync': False,
        }]
    )

    # ── 1b. USB Camera Node (DELETED) ───────────────────────────────────────

    # ── 2a. GPU pipeline for RealSense ────────────────────────────────────────
    # RectifyNode + AprilTagNode share a ComposableNodeContainer so that
    # NITROS zero-copy transport is used between them (no CPU copy needed).
    realsense_rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify_rs',
        namespace='',
        parameters=[{
            'output_width': 1280,
            'output_height': 720,
        }],
        remappings=[
            ('image_raw', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ]
    )

    realsense_apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag_rs',
        namespace='',
        parameters=[isaac_params_path],
        remappings=[
            ('image', 'image_rect'),
            ('camera_info', 'camera_info_rect'),
            ('tag_detections', '/tag_detections'),
        ]
    )

    realsense_apriltag_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('camera_type', 'realsense'),
        package='rclcpp_components',
        name='apriltag_gpu_container_rs',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            realsense_rectify_node,
            realsense_apriltag_node,
        ],
        output='screen'
    )

    # ── 2b. GPU pipeline for USB camera ──────────────────────────────────────
    usb_rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify_usb',
        namespace='',
        parameters=[{
            'output_width': 1280,
            'output_height': 720,
        }],
        remappings=[
            ('image_raw', '/usb_camera/image_raw'),
            ('camera_info', '/usb_camera/camera_info'),
        ]
    )

    usb_apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag_usb',
        namespace='',
        parameters=[isaac_params_path],
        remappings=[
            ('image', 'image_rect'),
            ('camera_info', 'camera_info_rect'),
            ('tag_detections', '/usb_camera/tag_detections'),
        ]
    )

    usb_apriltag_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('camera_type', 'usb'),
        package='rclcpp_components',
        name='apriltag_gpu_container_usb',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            usb_rectify_node,
            usb_apriltag_node,
        ],
        output='screen'
    )

    # ── 3. Visualizer Nodes ───────────────────────────────────────────────────
    visualizer_node_rs = Node(
        condition=LaunchConfigurationEquals('camera_type', 'realsense'),
        package='station_detection_APRILTAG',
        executable='apriltag_visualizer',
        name='apriltag_visualizer_rs',
        remappings=[
            ('/camera/image_raw', '/camera/camera/color/image_raw'),
            ('/tag_detections', '/tag_detections'),
            ('/camera/tag_detections_image/compressed', '/apriltag/overlay/compressed'),
        ]
    )

    visualizer_node_usb = Node(
        condition=LaunchConfigurationEquals('camera_type', 'usb'),
        package='station_detection_APRILTAG',
        executable='apriltag_visualizer',
        name='apriltag_visualizer_usb',
        remappings=[
            ('/camera/image_raw', '/usb_camera/image_raw'),
            ('/tag_detections', '/usb_camera/tag_detections'),
            ('/camera/tag_detections_image/compressed', '/usb_camera/tag_detections_image/compressed'),
        ]
    )


    return LaunchDescription([
        camera_type_arg,
        realsense_node,
        realsense_apriltag_container,
        usb_apriltag_container,
        visualizer_node_rs,
        visualizer_node_usb,
    ])
