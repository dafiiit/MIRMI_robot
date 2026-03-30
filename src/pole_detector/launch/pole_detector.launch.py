from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/livox/lidar',
        description='The LIDAR PointCloud2 topic name'
    )

    # Pole Detector Node
    pole_detector_node = Node(
        package='pole_detector',
        executable='pole_detector',
        name='pole_detector',
        parameters=[{
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            # You can add other parameters here if you'd like to expose them to the launch file
        }],
        output='screen'
    )

    # Pole Marker Node
    pole_marker_node = Node(
        package='pole_detector',
        executable='pole_marker',
        name='pole_marker',
        output='screen'
    )

    return LaunchDescription([
        cloud_topic_arg,
        pole_detector_node,
        pole_marker_node
    ])
