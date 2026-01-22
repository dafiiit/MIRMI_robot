from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_cmdvel_to_px4 = FindPackageShare('cmdvel_to_px4')
    pkg_foxglove_bridge = FindPackageShare('foxglove_bridge')
    pkg_mirmi_apriltag = FindPackageShare('mirmi_apriltag')

    # 1. Include existing px4_bridge.launch.py
    px4_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_cmdvel_to_px4, 'launch', 'px4_bridge.launch.py'])
        ])
    )

    # 2. Launch Foxglove Bridge
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([pkg_foxglove_bridge, 'launch', 'foxglove_bridge_launch.xml'])
        ])
    )

    # 3. Run Vicon Odometry Publisher
    vicon_odometry_node = Node(
        package='px4_localization_bridge',
        executable='vicon_odometry_pub',
        name='vicon_odometry_pub',
        output='screen',
        parameters=[{'vicon_topic': '/vicon/Robot_1/Robot_1/pose'}],
    )

    # 4. Run PX4 arm service command node
    px4_arm_service_node = Node(
        package='px4_arm_service',
        executable='command_services',
        name='command_services',
        output='screen',
    )

    # 5. Launch AprilTag Detection
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mirmi_apriltag, 'launch', 'usb_apriltag_detection.launch.py'])
        ])
    )

    # 6. Set Offboard Mode (Delayed to ensure service is ready)
    set_offboard_mode = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/px4/offboard',
                    'std_srvs/srv/SetBool',
                    '{data: true}'
                ],
                output='screen',
                shell=True
            )
        ]
    )

    return LaunchDescription([
        px4_bridge_launch,
        foxglove_bridge_launch,
        vicon_odometry_node,
        px4_arm_service_node,
        apriltag_launch,
        set_offboard_mode,
    ])
