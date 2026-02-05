#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsPositionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('gps_position_pub')

        self.declare_parameter('topic', '/gps/aux_global_position')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('frame_id', 'gps')

        self.declare_parameter('lat_deg', 0.0)
        self.declare_parameter('lon_deg', 0.0)
        self.declare_parameter('alt_m', 0.0)
        self.declare_parameter('alt_ellipsoid_m', math.nan)

        self.declare_parameter('status', NavSatStatus.STATUS_FIX)
        self.declare_parameter('service', NavSatStatus.SERVICE_GPS)

        self.declare_parameter('cov_xx', 2.5)
        self.declare_parameter('cov_yy', 2.5)
        self.declare_parameter('cov_zz', 4.0)

        topic = self.get_parameter('topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(NavSatFix, topic, qos)
        self.timer = self.create_timer(1.0 / max(0.1, rate_hz), self.timer_cb)

        self.get_logger().info(f"GpsPositionPublisher -> {topic} @ {rate_hz:.1f} Hz")

    def timer_cb(self) -> None:
        lat_deg = float(self.get_parameter('lat_deg').value)
        lon_deg = float(self.get_parameter('lon_deg').value)
        alt_m = float(self.get_parameter('alt_m').value)
        alt_ellipsoid_m = float(self.get_parameter('alt_ellipsoid_m').value)

        if math.isnan(alt_ellipsoid_m):
            alt_ellipsoid_m = alt_m

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

        msg.latitude = float(lat_deg)
        msg.longitude = float(lon_deg)
        msg.altitude = float(alt_ellipsoid_m)

        msg.status.status = int(self.get_parameter('status').value)
        msg.status.service = int(self.get_parameter('service').value)

        cov_xx = float(self.get_parameter('cov_xx').value)
        cov_yy = float(self.get_parameter('cov_yy').value)
        cov_zz = float(self.get_parameter('cov_zz').value)
        msg.position_covariance = [
            cov_xx, 0.0, 0.0,
            0.0, cov_yy, 0.0,
            0.0, 0.0, cov_zz,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
