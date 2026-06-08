#!/usr/bin/env python3
"""
dock_goal_sender — Navigate to a pose relative to the docking station.

Sends a NavigateToPose goal computed from:
  - docking station GPS position + orientation  (config/dock_station.yaml)
  - user-specified distance [m] and angle [deg] around the station
  - live robot GPS fix for the ENU datum                (same as gps_goal_sender)

Angle convention (matches the sweep test data):
  0°   = directly in front of the station  (along the station's 0° axis)
  CCW  = positive angles (30°, 45°, 90°, 135°, 180° = behind station)
  Robot always faces the station at the target pose.

Usage:
  ros2 topic pub --once /dock_goal geometry_msgs/msg/Point \
    "{x: 4.0, y: 45.0, z: 0.0}"
  # x = distance [m],  y = angle [deg],  z = unused

The node loads dock_station.yaml from the navigation package's config dir.
"""

import math
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point, PoseStamped
from px4_msgs.msg import VehicleGlobalPosition
from nav2_msgs.action import NavigateToPose


def gps_to_enu(lat, lon, datum_lat, datum_lon):
    """Flat-Earth GPS → ENU (east_m, north_m) relative to datum."""
    R = 6_371_000.0
    east  = R * math.radians(lon - datum_lon) * math.cos(math.radians(datum_lat))
    north = R * math.radians(lat - datum_lat)
    return east, north


class DockGoalSender(Node):
    def __init__(self):
        super().__init__('dock_goal_sender')

        # ── Load dock station config ──────────────────────────────────────
        config_path = os.path.join(
            get_package_share_directory('navigation'), 'config', 'dock_station.yaml'
        )
        with open(config_path) as f:
            cfg = yaml.safe_load(f)['dock_station']

        self._station_lat     = cfg['lat']
        self._station_lon     = cfg['lon']
        self._heading_enu_rad = math.radians(cfg['heading_enu_deg'])

        self.get_logger().info(
            f'Dock station: lat={self._station_lat:.8f} lon={self._station_lon:.8f} '
            f'heading={cfg["heading_compass_deg"]:.1f}° compass'
        )

        # ── Datum (same auto-set logic as gps_goal_sender) ────────────────
        # Optional override via parameters
        self.declare_parameter('datum_lat', 0.0)
        self.declare_parameter('datum_lon', 0.0)
        dl = self.get_parameter('datum_lat').value
        do = self.get_parameter('datum_lon').value
        if dl != 0.0 or do != 0.0:
            self._datum = (dl, do)
            self.get_logger().info(f'Using manual datum: lat={dl:.7f} lon={do:.7f}')
        else:
            self._datum = None
            self.get_logger().info(
                'Datum not set — will auto-set from first /fmu/out/vehicle_global_position fix.'
            )

        # ── PX4 GPS subscriber ────────────────────────────────────────────
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._gps_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self._gps_cb,
            px4_qos,
        )

        # ── Goal input ────────────────────────────────────────────────────
        # geometry_msgs/Point: x = distance [m], y = angle [deg], z = unused
        self._goal_sub = self.create_subscription(
            Point,
            '/dock_goal',
            self._goal_cb,
            10,
        )

        # ── nav2 action client ────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            'dock_goal_sender ready.\n'
            '  Publish to /dock_goal: geometry_msgs/Point {x: <distance_m>, y: <angle_deg>}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _gps_cb(self, msg: VehicleGlobalPosition):
        if self._datum is None and msg.lat_lon_valid:
            self._datum = (msg.lat, msg.lon)
            self.get_logger().info(
                f'Datum auto-set: lat={msg.lat:.7f} lon={msg.lon:.7f}'
            )

    def _goal_cb(self, msg: Point):
        if self._datum is None:
            self.get_logger().error(
                'No GPS datum yet — robot GPS fix not received. '
                'Move the robot outdoors or set datum_lat/datum_lon parameters.'
            )
            return

        distance_m = float(msg.x)
        angle_deg  = float(msg.y)
        angle_rad  = math.radians(angle_deg)

        datum_lat, datum_lon = self._datum

        # Station position in ENU from datum
        st_east, st_north = gps_to_enu(
            self._station_lat, self._station_lon,
            datum_lat, datum_lon,
        )

        # Target position: D metres from station along (station_heading + angle)
        direction_rad = self._heading_enu_rad + angle_rad
        target_east  = st_east  + distance_m * math.cos(direction_rad)
        target_north = st_north + distance_m * math.sin(direction_rad)

        # Robot faces the station from the target → heading is opposite to direction
        robot_yaw_enu = direction_rad + math.pi  # pointing back toward station

        self.get_logger().info(
            f'Dock goal  dist={distance_m:.1f}m  angle={angle_deg:.1f}°\n'
            f'  Station ENU:  ({st_east:.2f}, {st_north:.2f})\n'
            f'  Target ENU:   ({target_east:.2f}, {target_north:.2f})\n'
            f'  Robot heading: {math.degrees(robot_yaw_enu):.1f}° ENU'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateToPose action server not available — is nav2 running?'
            )
            return

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = target_east
        pose.pose.position.y = target_north
        pose.pose.position.z = 0.0
        # Quaternion from yaw (rotation around Z in ENU)
        pose.pose.orientation.z = math.sin(robot_yaw_enu / 2.0)
        pose.pose.orientation.w = math.cos(robot_yaw_enu / 2.0)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        future = self._nav_client.send_goal_async(
            nav_goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by nav2.')
            return
        self.get_logger().info('Goal accepted — navigating to dock position.')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0
        )

    def _result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Reached dock position.')
        else:
            self.get_logger().warn(f'Navigation ended with status {status}.')


def main(args=None):
    rclpy.init(args=args)
    node = DockGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
