#!/usr/bin/env python3
"""
Bridge Vicon PoseStamped to PX4 VehicleOdometry on /fmu/in/vehicle_odometry.

Assumptions:
- Vicon publishes PoseStamped in ENU/world with body axes FLU.
- PX4 expects NED with body axes FRD.
Adjust the frame swap below if your Vicon frame differs.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry as VVO


# Quaternion helpers (w, x, y, z)
def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


# ENU->NED, FLU->FRD quaternions (w, x, y, z)
Q_ENU2NED = (0.0, 0.70710678, 0.70710678, 0.0)
Q_FLU2FRD = (0.0, 1.0, 0.0, 0.0)


def enu_flu_to_ned_frd(q):
    """Convert quaternion from ENU/FLU to NED/FRD."""
    return quat_multiply(Q_FLU2FRD, quat_multiply(Q_ENU2NED, q))


class ViconOdometryPublisher(Node):
    """Subscribe to Vicon PoseStamped and publish VehicleOdometry for PX4 EKF2."""

    def __init__(self):
        super().__init__('vicon_odometry_pub')

        self.declare_parameter('vicon_topic', '/vicon/Robot_1/Robot_1/pose')

        self.sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('vicon_topic').get_parameter_value().string_value,
            self.pose_cb,
            50,
        )

        self.pub = self.create_publisher(VVO, '/fmu/in/vehicle_visual_odometry', 50)

        self.last_pos_ned = None
        self.last_stamp_us = None

        self.get_logger().info('ViconOdometryPublisher -> /fmu/in/vehicle_visual_odometry')

    def pose_cb(self, msg_in: PoseStamped):
        # Time
        t_sample_us = msg_in.header.stamp.sec * 1_000_000 + msg_in.header.stamp.nanosec // 1000
        t_now_us = self.get_clock().now().nanoseconds // 1000

        # Position: ENU (Vicon) -> NED (PX4)
        p = msg_in.pose.position
        x_n = p.y
        y_e = p.x
        z_d = -p.z

        # Orientation: ENU/FLU -> NED/FRD
        q = msg_in.pose.orientation
        qw, qx, qy, qz = enu_flu_to_ned_frd((q.w, q.x, q.y, q.z))

        # Linear velocity from finite differences in NED
        vx = vy = vz = 0.0
        if self.last_pos_ned is not None and self.last_stamp_us is not None:
            dt = (t_sample_us - self.last_stamp_us) * 1e-6
            if dt > 1e-4:
                vx = (x_n - self.last_pos_ned[0]) / dt
                vy = (y_e - self.last_pos_ned[1]) / dt
                vz = (z_d - self.last_pos_ned[2]) / dt

        m = VVO()
        m.timestamp = t_now_us
        m.timestamp_sample = t_sample_us
        m.pose_frame = VVO.POSE_FRAME_NED
        m.velocity_frame = VVO.VELOCITY_FRAME_NED
        m.position = [x_n, y_e, z_d]
        m.q = [qw, qx, qy, qz]
        m.velocity = [vx, vy, vz]
        m.angular_velocity = [0.0, 0.0, 0.0]
        m.position_variance = [0.01, 0.01, 0.01]
        m.orientation_variance = [0.01, 0.01, 0.01]
        m.velocity_variance = [0.02, 0.02, 0.02]
        m.quality = 100
        m.reset_counter = 0

        self.pub.publish(m)

        self.last_pos_ned = (x_n, y_e, z_d)
        self.last_stamp_us = t_sample_us


def main(args=None):
    rclpy.init(args=args)
    node = ViconOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
