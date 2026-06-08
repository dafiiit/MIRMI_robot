#!/usr/bin/env python3
"""
gps_goal_sender — Convert a GPS coordinate into a nav2 NavigateToPose goal.

Usage (after launch):
  ros2 topic pub --once /navigate_gps sensor_msgs/msg/NavSatFix \
    "{latitude: 48.1234, longitude: 11.5678, altitude: 0.0}"

How the conversion works:
  1. The robot's GPS position at the moment the node sets its datum is treated
     as the origin of the map frame (East = +X, North = +Y, REP-103).
  2. The flat-Earth ENU approximation converts lat/lon deltas to metres.
     Error is <1 m over 10 km, which is more than enough for this use case.
  3. The ENU (east, north) offset is sent directly as a NavigateToPose goal.

Datum can be:
  a) Auto-set: first /fmu/out/vehicle_global_position fix received → stored as datum.
  b) Manual:   set ROS parameters datum_lat / datum_lon / datum_alt before starting.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix
from px4_msgs.msg import VehicleGlobalPosition
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


def gps_to_enu(lat, lon, alt, datum_lat, datum_lon, datum_alt):
    """
    Flat-Earth WGS84 → ENU (East-North-Up) relative to datum.
    Returns (east_m, north_m, up_m).
    """
    R = 6_371_000.0
    dlat = math.radians(lat - datum_lat)
    dlon = math.radians(lon - datum_lon)
    east  = R * dlon * math.cos(math.radians(datum_lat))
    north = R * dlat
    up    = alt - datum_alt
    return east, north, up


class GpsGoalSender(Node):
    def __init__(self):
        super().__init__('gps_goal_sender')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('datum_lat',  0.0)
        self.declare_parameter('datum_lon',  0.0)
        self.declare_parameter('datum_alt',  0.0)
        # goal_yaw_deg: heading at the target in degrees (0 = East, 90 = North).
        # Leave at -1 to let nav2 decide the final heading.
        self.declare_parameter('goal_yaw_deg', -1.0)

        datum_lat = self.get_parameter('datum_lat').value
        datum_lon = self.get_parameter('datum_lon').value
        datum_alt = self.get_parameter('datum_alt').value

        # Datum is "set" if user provided non-zero values
        if datum_lat != 0.0 or datum_lon != 0.0:
            self._datum = (datum_lat, datum_lon, datum_alt)
            self.get_logger().info(
                f'Using manual datum: lat={datum_lat:.7f} lon={datum_lon:.7f} alt={datum_alt:.1f} m'
            )
        else:
            self._datum = None
            self.get_logger().info(
                'No datum set — will auto-set from first /fmu/out/vehicle_global_position fix.'
            )

        # ── PX4 GPS subscriber (for auto-datum) ───────────────────────────
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

        # ── Goal input subscriber ──────────────────────────────────────────
        # Publish a NavSatFix here to trigger navigation.
        self._goal_sub = self.create_subscription(
            NavSatFix,
            '/navigate_gps',
            self._goal_cb,
            10,
        )

        # ── nav2 action client ────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            'gps_goal_sender ready. '
            'Publish a NavSatFix to /navigate_gps to start navigation.'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _gps_cb(self, msg: VehicleGlobalPosition):
        if self._datum is None and msg.lat_lon_valid:
            self._datum = (msg.lat, msg.lon, float(msg.alt))
            self.get_logger().info(
                f'Datum auto-set: lat={msg.lat:.7f} lon={msg.lon:.7f} alt={msg.alt:.1f} m'
            )

    def _goal_cb(self, msg: NavSatFix):
        if self._datum is None:
            self.get_logger().error(
                'No datum yet — robot GPS fix not received. '
                'Drive the robot outdoors so PX4 gets a GPS fix, '
                'or set datum_lat/datum_lon parameters.'
            )
            return

        datum_lat, datum_lon, datum_alt = self._datum
        east, north, _ = gps_to_enu(
            msg.latitude, msg.longitude, msg.altitude,
            datum_lat, datum_lon, datum_alt,
        )

        self.get_logger().info(
            f'GPS goal  lat={msg.latitude:.7f} lon={msg.longitude:.7f} → '
            f'map ({east:.2f} m East, {north:.2f} m North)'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateToPose action server not available. Is nav2 running?'
            )
            return

        goal_yaw_deg = self.get_parameter('goal_yaw_deg').value

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = east
        pose.pose.position.y = north
        pose.pose.position.z = 0.0

        if goal_yaw_deg >= 0.0:
            # Convert requested heading to quaternion (rotation around Z)
            yaw = math.radians(goal_yaw_deg)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            # nav2 will align heading towards the goal automatically
            pose.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        send_future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by nav2.')
            return
        self.get_logger().info('Goal accepted — robot is navigating.')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist:.2f} m', throttle_duration_sec=2.0)

    def _result_cb(self, future):
        result = future.result()
        if result.status == 4:   # SUCCEEDED
            self.get_logger().info('Navigation succeeded — goal reached.')
        else:
            self.get_logger().warn(f'Navigation finished with status {result.status}.')


def main(args=None):
    rclpy.init(args=args)
    node = GpsGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
