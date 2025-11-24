#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

def set_if(msg, name, value):
    if hasattr(msg, name):
        setattr(msg, name, value)

class OdomToPX4(Node):
    """
    Konvertiert nav_msgs/Odometry (/odom) in PX4 VehicleOdometry
    und publiziert nach /fmu/in/vehicle_odometry.
    """
    def __init__(self):
        super().__init__('odom_to_px4')
        self.pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_odometry', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 30)
        self.get_logger().info('odom_to_px4: /odom -> /fmu/in/vehicle_odometry')

    def cb(self, odom: Odometry):
        now_us = self.get_clock().now().nanoseconds // 1000

        msg = VehicleOdometry()
        set_if(msg, 'timestamp', now_us)
        # sample-Zeit: nimm Zeit aus /odom falls gewünscht
        set_if(msg, 'timestamp_sample',
               odom.header.stamp.sec * 1000000 + odom.header.stamp.nanosec // 1000
               if odom.header.stamp.sec or odom.header.stamp.nanosec else now_us)

        # Position
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        if hasattr(msg, 'position'):
            msg.position = [x, y, z]
        else:
            set_if(msg, 'x', x); set_if(msg, 'y', y); set_if(msg, 'z', z)

        # Orientierung (Quaternion)
        q = odom.pose.pose.orientation
        if hasattr(msg, 'q'):
            msg.q = [q.x, q.y, q.z, q.w]
        else:
            set_if(msg, 'q', [q.x, q.y, q.z, q.w])

        # Linear velocity
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z
        if hasattr(msg, 'velocity'):
            msg.velocity = [vx, vy, vz]
        else:
            set_if(msg, 'vx', vx); set_if(msg, 'vy', vy); set_if(msg, 'vz', vz)

        # Angular velocity
        wx = odom.twist.twist.angular.x
        wy = odom.twist.twist.angular.y
        wz = odom.twist.twist.angular.z
        if hasattr(msg, 'angular_velocity'):
            msg.angular_velocity = [wx, wy, wz]
        else:
            set_if(msg, 'rollspeed', wx)
            set_if(msg, 'pitchspeed', wy)
            set_if(msg, 'yawspeed', wz)

        # Variance/Covariance (minimal diagonal angeben, falls Felder existieren)
        if hasattr(msg, 'position_variance'):
            msg.position_variance = [odom.pose.covariance[0], odom.pose.covariance[7], odom.pose.covariance[14]]
        if hasattr(msg, 'orientation_variance'):
            # grob aus Pose-Cov ableiten oder konstant ansetzen
            msg.orientation_variance = [0.02, 0.02, 0.05]
        if hasattr(msg, 'velocity_variance'):
            msg.velocity_variance = [odom.twist.covariance[0], odom.twist.covariance[7], odom.twist.covariance[14]]

        # Frames (falls vorhanden)
        set_if(msg, 'pose_frame', 1)      # LOCAL_FRAME_NED
        set_if(msg, 'velocity_frame', 1)

        # Optional
        set_if(msg, 'quality', 100)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToPX4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
