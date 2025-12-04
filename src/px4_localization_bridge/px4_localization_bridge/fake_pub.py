#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry as VO


class FakeVIO(Node):
    """
    Stationäre Fake-VIO für EKF2 (30–50 Hz), Frames/Kovarianzen korrekt gesetzt.
    Sende NED-Pose (0,0,0) mit fixer Yaw (parametrierbar), v = 0.
    Topic: /fmu/in/vehicle_visual_odometry
    """
    def __init__(self):
        super().__init__('fake_vio')
        self.pub = self.create_publisher(VO, '/fmu/in/vehicle_visual_odometry', 10)

        # Parameter
        self.declare_parameter('rate_hz', 33.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('quality', 100)
        self.declare_parameter('pos_var', [0.02, 0.02, 0.05])
        self.declare_parameter('ori_var', [0.01, 0.01, 0.003])
        self.declare_parameter('vel_var', [0.02, 0.02, 0.05])

        self.rate = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / max(1.0, self.rate)
        self.yaw = math.radians(float(self.get_parameter('yaw_deg').value))

        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info(
            f'FakeVIO → /fmu/in/vehicle_visual_odometry @ {self.rate:.1f} Hz'
        )

    def timer_cb(self):
        now_us = self.get_clock().now().nanoseconds // 1000
        msg = VO()
        msg.timestamp = now_us
        msg.timestamp_sample = now_us

        # Frames: NED für Pose & Velocity (konsistent und simpel)
        msg.pose_frame = VO.POSE_FRAME_NED        # 1
        msg.velocity_frame = VO.VELOCITY_FRAME_NED  # 1

        # Position/Velocity: 0
        msg.position = [0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0]

        # Orientation: Quaternion [w, x, y, z] aus Yaw
        half = 0.5 * self.yaw
        w = math.cos(half)
        z = math.sin(half)
        msg.q = [w, 0.0, 0.0, z]

        # Angular velocity (rad/s)
        msg.angular_velocity = [0.0, 0.0, 0.0]

        # Variances (falls Felder vorhanden)
        if hasattr(msg, 'position_variance'):
            msg.position_variance = list(self.get_parameter('pos_var').value)
        if hasattr(msg, 'orientation_variance'):
            msg.orientation_variance = list(self.get_parameter('ori_var').value)
        if hasattr(msg, 'velocity_variance'):
            msg.velocity_variance = list(self.get_parameter('vel_var').value)

        # Qualität / Reset Counter (optional)
        if hasattr(msg, 'quality'):
            msg.quality = int(self.get_parameter('quality').value)
        if hasattr(msg, 'reset_counter'):
            msg.reset_counter = 0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVIO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

