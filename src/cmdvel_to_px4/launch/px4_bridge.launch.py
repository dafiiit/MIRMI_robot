from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    vicon_odometry = Node(
        package='px4_localization_bridge',
        executable='vicon_odometry_pub',
        name='vicon_odometry_pub',
        output='screen',
        parameters=[{'vicon_topic': '/vicon/Robot_1/Robot_1/pose'}],
    )
    
    fake_odometry = Node(
        package='px4_localization_bridge',
        executable='fake_odometry_pub_2',
        name='fake_odometry_pub_2',
        output='screen',
    )

    cmdvel = Node(
        package='cmdvel_to_px4',
        executable='cmdvel_to_px4_3',
        name='cmdvel_to_px4_3',
        output='screen',
    )

    litime_bms = Node(
        package='litime_bms_ros',
        executable='litime_bms_ros_node',
        name='litime_bms_ros_node',
        output='screen',
    )

    return LaunchDescription([
        #vicon_odometry,
        #fake_odometry,
        litime_bms,
        TimerAction(period=3.0, actions=[cmdvel]),
    ])
