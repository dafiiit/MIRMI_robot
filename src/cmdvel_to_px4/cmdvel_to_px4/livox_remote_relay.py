#!/usr/bin/env python3

from math import isfinite

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LivoxRemoteRelay(Node):
    def __init__(self):
        super().__init__('livox_remote_relay')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_remote')
        self.declare_parameter('decimation_stride', 4)
        self.declare_parameter('output_hz', 2.0)
        self.declare_parameter('skip_nans', True)
        self.declare_parameter('min_range', 0.0)
        self.declare_parameter('max_range', 0.0)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            qos,
        )

        self.last_publish_time = None
        self.input_frames = 0
        self.output_frames = 0
        self.debug_counter = 0

        self.get_logger().info(
            f"Relaying {input_topic} -> {output_topic} "
            f"with stride={self.get_stride()} output_hz={self.get_output_hz():.2f}"
        )

    def get_stride(self):
        stride = self.get_parameter('decimation_stride').get_parameter_value().integer_value
        return max(1, stride)

    def get_output_hz(self):
        hz = self.get_parameter('output_hz').get_parameter_value().double_value
        return hz if hz > 0.0 else 0.0

    def get_min_range(self):
        return self.get_parameter('min_range').get_parameter_value().double_value

    def get_max_range(self):
        return self.get_parameter('max_range').get_parameter_value().double_value

    def should_publish_now(self):
        output_hz = self.get_output_hz()
        if output_hz <= 0.0:
            return True

        now = self.get_clock().now()
        min_period = Duration(seconds=1.0 / output_hz)
        if self.last_publish_time is None or (now - self.last_publish_time) >= min_period:
            self.last_publish_time = now
            return True
        return False

    def pointcloud_callback(self, msg: PointCloud2):
        self.input_frames += 1
        if not self.should_publish_now():
            return

        stride = self.get_stride()
        skip_nans = self.get_parameter('skip_nans').get_parameter_value().bool_value
        min_range = self.get_min_range()
        max_range = self.get_max_range()

        sampled_points = []
        total_points = 0
        kept_points = 0

        for point in point_cloud2.read_points(msg, field_names=None, skip_nans=skip_nans):
            total_points += 1
            if total_points % stride != 1:
                continue

            x = point[0]
            y = point[1]
            z = point[2]
            if not (isfinite(x) and isfinite(y) and isfinite(z)):
                continue

            distance_sq = x * x + y * y + z * z
            if min_range > 0.0 and distance_sq < min_range * min_range:
                continue
            if max_range > 0.0 and distance_sq > max_range * max_range:
                continue

            sampled_points.append(tuple(point))
            kept_points += 1

        remote_msg = point_cloud2.create_cloud(msg.header, msg.fields, sampled_points)
        remote_msg.is_dense = skip_nans
        self.publisher.publish(remote_msg)
        self.output_frames += 1

        self.debug_counter = (self.debug_counter + 1) % 20
        if self.debug_counter == 0:
            ratio = (kept_points / total_points) if total_points else 0.0
            self.get_logger().info(
                f"remote cloud points={kept_points}/{total_points} ({ratio:.2%}) "
                f"frames_in={self.input_frames} frames_out={self.output_frames}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LivoxRemoteRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

