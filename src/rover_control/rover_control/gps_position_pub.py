#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleGlobalPosition


class GpsPositionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('gps_position_pub')

        self.declare_parameter('topic', '/fmu/in/aux_global_position')
        self.declare_parameter('rate_hz', 5.0)

        self.declare_parameter('lat_deg', 0.0)
        self.declare_parameter('lon_deg', 0.0)
        self.declare_parameter('alt_m', 0.0)
        self.declare_parameter('alt_ellipsoid_m', math.nan)

        self.declare_parameter('eph', 0.5)
        self.declare_parameter('epv', 0.8)
        self.declare_parameter('terrain_alt_m', 0.0)
        self.declare_parameter('terrain_alt_valid', False)
        self.declare_parameter('dead_reckoning', False)

        topic = self.get_parameter('topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(VehicleGlobalPosition, topic, qos)
        self.timer = self.create_timer(1.0 / max(0.1, rate_hz), self.timer_cb)

        self.get_logger().info(f"GpsPositionPublisher -> {topic} @ {rate_hz:.1f} Hz")

    def timer_cb(self) -> None:
        now_us = self.get_clock().now().nanoseconds // 1000

        lat_deg = float(self.get_parameter('lat_deg').value)
        lon_deg = float(self.get_parameter('lon_deg').value)
        alt_m = float(self.get_parameter('alt_m').value)
        alt_ellipsoid_m = float(self.get_parameter('alt_ellipsoid_m').value)

        if math.isnan(alt_ellipsoid_m):
            alt_ellipsoid_m = alt_m

        msg = VehicleGlobalPosition()
        msg.timestamp = now_us
        msg.timestamp_sample = now_us

        msg.lat = float(lat_deg)
        msg.lon = float(lon_deg)
        msg.alt = float(alt_m)
        msg.alt_ellipsoid = float(alt_ellipsoid_m)

        msg.delta_alt = 0.0
        msg.lat_lon_reset_counter = 0
        msg.alt_reset_counter = 0

        msg.eph = float(self.get_parameter('eph').value)
        msg.epv = float(self.get_parameter('epv').value)
        msg.terrain_alt = float(self.get_parameter('terrain_alt_m').value)
        msg.terrain_alt_valid = bool(self.get_parameter('terrain_alt_valid').value)
        msg.dead_reckoning = bool(self.get_parameter('dead_reckoning').value)

        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
