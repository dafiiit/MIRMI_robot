import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    isaac_params_path = os.path.join(
        get_package_share_directory('station_detection_APRILTAG'),
        'config',
        'isaac_ros_apriltag_params.yaml'
    )

    # GPU pipeline: RectifyNode + AprilTagNode in one container.
    # NITROS zero-copy transport is used between nodes in the same container.
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[isaac_params_path],
        remappings=[
            ('image', 'image_rect'),
            ('camera_info', 'camera_info_rect'),
        ]
    )

    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_gpu_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
        ],
        output='screen'
    )

    # Visualizer (CPU Python node, subscribes to GPU detections on /tag_detections)
    visualizer_node = Node(
        package='station_detection_APRILTAG',
        executable='apriltag_visualizer',
        name='apriltag_visualizer',
    )

    return LaunchDescription([
        apriltag_container,
        visualizer_node,
    ])