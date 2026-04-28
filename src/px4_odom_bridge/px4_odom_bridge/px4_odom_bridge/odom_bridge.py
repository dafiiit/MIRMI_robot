#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PX4OdomBridge(Node):
    def __init__(self):
        super().__init__('px4_odom_bridge')

        self.last_pub_time = self.get_clock().now()
        self.pub_period = 1.0 / 50.0  # 50 Hz

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.callback,
            qos_profile
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odom_px4',
            qos_profile
        )
        
        self.get_logger().info('PX4 Odom Bridge Started. Initializing with Diagonal Covariance.')

    def callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_pub_time).nanoseconds * 1e-9
        if dt < self.pub_period:
            return
        self.last_pub_time = now

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link_ekf'

        # Rounding to 5 decimal places prevents floating point "jitter" and checking for math.isnan() ( NaN poisons the ekf and makes it unable to work)
        try:
            px = round(float(msg.position[0]), 5)
            py = round(-float(msg.position[1]), 5)
            
            vx = round(float(msg.velocity[0]), 5)
            vy = round(-float(msg.velocity[1]), 5)

            # Check if any input is NaN before building the message
            if any(math.isnan(val) for val in [px, py, vx, vy]):
                return 
        except (ValueError, TypeError):
            return

        # 1. Make Z an irrelevant metric to avoid adding noise.
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = 0.0 

        # 2. Orientation Handling & Normalization
        q = [float(msg.q[0]), float(msg.q[1]), -float(msg.q[2]), -float(msg.q[3])]
        norm = math.sqrt(sum(i**2 for i in q))
        
        if norm > 1e-6:
            odom.pose.pose.orientation.w = q[0] / norm
            odom.pose.pose.orientation.x = q[1] / norm
            odom.pose.pose.orientation.y = q[2] / norm
            odom.pose.pose.orientation.z = q[3] / norm
        else:
            odom.pose.pose.orientation.w = 1.0

        # 3. Velocity Mapping
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0 
        
        # Explicitly zero out angular velocities
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # 4. Diagonal Covariance Fix
        diag_val = 0.05
        cov = [0.0] * 36
        for i in [0, 7, 14, 21, 28, 35]:
            cov[i] = diag_val
        
        odom.pose.covariance = cov
        odom.twist.covariance = cov

        self.pub.publish(odom)
        
def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()