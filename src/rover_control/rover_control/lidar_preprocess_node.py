#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2


class LidarPreprocessNode(Node):
    """Convert PointCloud2 to fixed-size polar bins as LaserScan.

    For each angular bin, publishes the minimum XY distance of points
    that pass z/r filters.
    """

    def __init__(self) -> None:
        super().__init__("lidar_preprocess_node")

        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("scan_topic", "/lidar_polar")

        self.declare_parameter("num_bins", 360)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)

        self.declare_parameter("z_min", -0.2)
        self.declare_parameter("z_max", 1.5)
        self.declare_parameter("r_min", 0.05)
        self.declare_parameter("r_max", 10.0)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)

        self.sub = self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 10)
        self.pub = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.get_logger().info(f"LidarPreprocessNode: {self.cloud_topic} -> {self.scan_topic}")

    def on_cloud(self, msg: PointCloud2) -> None:
        num_bins = int(self.get_parameter("num_bins").value)
        angle_min = float(self.get_parameter("angle_min").value)
        angle_max = float(self.get_parameter("angle_max").value)
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)
        r_min = float(self.get_parameter("r_min").value)
        r_max = float(self.get_parameter("r_max").value)

        if num_bins <= 1:
            self.get_logger().error("num_bins must be > 1")
            return

        ranges = np.full((num_bins,), np.inf, dtype=np.float32)
        inv_step = num_bins / max((angle_max - angle_min), 1e-6)

        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if z < z_min or z > z_max:
                continue

            r = math.hypot(x, y)
            if r < r_min or r > r_max:
                continue

            a = math.atan2(y, x)
            if a < angle_min or a >= angle_max:
                continue

            i = int((a - angle_min) * inv_step)
            if 0 <= i < num_bins and r < ranges[i]:
                ranges[i] = r

        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = float(angle_min)
        scan.angle_max = float(angle_max)
        scan.angle_increment = float((angle_max - angle_min) / num_bins)
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = float(r_min)
        scan.range_max = float(r_max)

        # Keep inf for empty bins; downstream logger sanitizes.
        scan.ranges = ranges.tolist()
        scan.intensities = []

        self.pub.publish(scan)


def main() -> None:
    rclpy.init()
    node = LidarPreprocessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
