"""
navigation.launch.py

Launches the full navigation stack for the tracked AGV:
  1. robot_localization EKF  — fuses PX4 odom → /odometry/filtered + odom_filtered→base_link_ekf TF
  2. static TF               — base_link_ekf → livox_frame (LIDAR mounting, identity)
  3. pointcloud_to_laserscan — /_livox/lidar (PointCloud2) → /scan (LaserScan)
  4. rtabmap                 — 2D LIDAR SLAM → /map + map→odom_filtered TF
  5. nav2 nodes              — planner, controller (RPP), velocity_smoother, bt_navigator, …
  6. nav2 lifecycle_manager  — activates all nav2 nodes

Prerequisite: the Livox LIDAR driver must already be running (/_livox/lidar topic live).

TF tree:
  map
   └── odom_filtered          (rtabmap)
        └── base_link_ekf     (robot_localization EKF)
             └── livox_frame  (static, identity)
             └── camera_link  (URDF / robot_state_publisher)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    rtabmap_params = os.path.join(nav_share, 'config', 'rtabmap_params.yaml')
    nav2_params   = os.path.join(nav_share, 'config', 'nav2_params.yaml')
    ekf_launch    = os.path.expanduser('~/robot_localization/ekf_launch.py')

    # ── Launch arguments ──────────────────────────────────────────────────
    declare_gps_node = DeclareLaunchArgument(
        'use_gps_goal', default_value='true',
        description='Start the GPS-to-nav2-goal converter node'
    )
    declare_dock_node = DeclareLaunchArgument(
        'use_dock_goal', default_value='true',
        description='Start the dock-relative goal sender node'
    )
    declare_datum_lat = DeclareLaunchArgument(
        'datum_lat', default_value='0.0',
        description='Latitude of the map origin (0 = auto-set from first GPS fix)'
    )
    declare_datum_lon = DeclareLaunchArgument(
        'datum_lon', default_value='0.0',
        description='Longitude of the map origin (0 = auto-set from first GPS fix)'
    )
    declare_datum_alt = DeclareLaunchArgument(
        'datum_alt', default_value='0.0',
        description='Altitude of the map origin in metres'
    )

    # ── 1. robot_localization EKF ──────────────────────────────────────────
    # Fuses /odom_px4 → /odometry/filtered and broadcasts odom_filtered→base_link_ekf TF
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch)
    )

    # ── 2. Static TF: base_link_ekf → livox_frame ─────────────────────────
    # The Livox MID360 is mounted at the robot's centre of mass (identity transform).
    # Adjust xyz/rpy if the LIDAR is physically offset.
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link_ekf',
                   '--child-frame-id', 'livox_frame'],
    )

    # ── 3. PointCloud2 → LaserScan ────────────────────────────────────────
    # Converts the 3D Livox cloud to a 2D horizontal scan.
    # min/max_height define the vertical slice (relative to livox_frame).
    # Livox MID360 scans hemispherically — we take a ±15 cm band to get a
    # clean ground-level ring without floor returns.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.1,
            # MID360 vertical FOV: −7° to +52° (mostly upward, non-repetitive).
            # Only 7° below horizontal → floor returns appear very close to the robot.
            # Slice: start 10 cm above sensor (avoids floor) up to 2 m (catches walls,
            # people, obstacles). Adjust min_height DOWN if you need to detect low objects,
            # UP if you still see floor noise.
            'min_height':  0.10,   # 10 cm above livox_frame (floor-return-free zone)
            'max_height':  2.00,   # 2 m above livox_frame (catches walls, people)
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.00873,   # 0.5° resolution
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 20.0,
            'use_inf': False,      # drop sky/no-return beams (avoids Foxglove ∞ error)
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),   # livox driver publishes here (no underscore)
            ('scan',     '/scan'),
        ],
    )

    # ── 4. rtabmap (2D LIDAR SLAM) ────────────────────────────────────────
    # Subscribes to /scan + /odometry/filtered.
    # Publishes /map (OccupancyGrid) and the map→odom_filtered TF.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=[
            ('scan',  '/scan'),
            ('odom',  '/odometry/filtered'),
        ],
        arguments=['--delete_db_on_start'],  # remove for persistent maps
    )

    # ── 5a. controller_server ─────────────────────────────────────────────
    # Raw cmd_vel → cmd_vel_nav (velocity_smoother picks it up)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    # ── 5b. velocity_smoother ─────────────────────────────────────────────
    # Reads cmd_vel_nav, applies accel limits, publishes /cmd_vel (to PX4 bridge)
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
            ('cmd_vel',          'cmd_vel_nav'),   # input from controller_server
            ('cmd_vel_smoothed', '/cmd_vel'),       # output to cmdvel_to_px4
        ],
    )

    # ── 5c. planner_server ────────────────────────────────────────────────
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── 5d. behavior_server ───────────────────────────────────────────────
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    # ── 5e. bt_navigator ──────────────────────────────────────────────────
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    # ── 5f. waypoint_follower ─────────────────────────────────────────────
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params],
    )

    # ── 6. nav2 lifecycle_manager ─────────────────────────────────────────
    # Manages the lifecycle (configure → activate) of all nav2 nodes.
    # Does NOT include map_server or amcl — rtabmap handles both.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'velocity_smoother',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    # ── 7. Dock-relative goal sender ──────────────────────────────────────
    # Publish to /dock_goal: geometry_msgs/Point {x: distance_m, y: angle_deg}
    # Converts to map-frame NavigateToPose using dock_station.yaml config.
    dock_goal_sender = Node(
        package='navigation',
        executable='dock_goal_sender',
        name='dock_goal_sender',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_dock_goal')),
        parameters=[{
            'datum_lat': LaunchConfiguration('datum_lat'),
            'datum_lon': LaunchConfiguration('datum_lon'),
        }],
    )

    # ── 8. GPS → nav2 goal converter ──────────────────────────────────────
    # Publish a NavSatFix to /navigate_gps to trigger GPS-based navigation.
    # Datum is set from the first GPS fix unless datum_lat/lon params are given.
    gps_goal_sender = Node(
        package='navigation',
        executable='gps_goal_sender',
        name='gps_goal_sender',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gps_goal')),
        parameters=[{
            'datum_lat': LaunchConfiguration('datum_lat'),
            'datum_lon': LaunchConfiguration('datum_lon'),
            'datum_alt': LaunchConfiguration('datum_alt'),
        }],
    )

    return LaunchDescription([
        declare_gps_node,
        declare_dock_node,
        declare_datum_lat,
        declare_datum_lon,
        declare_datum_alt,
        ekf,
        static_tf_lidar,
        pointcloud_to_laserscan,
        rtabmap,
        controller_server,
        velocity_smoother,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        dock_goal_sender,
        gps_goal_sender,
    ])
