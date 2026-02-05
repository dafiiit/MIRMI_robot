#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleOdometry


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quat_wxyz(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CircleCmdVelPX4(Node):
    def __init__(self) -> None:
        super().__init__('circle_cmd_vel_px4')

        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_circle') 
        self.declare_parameter('center_x', -2.0)
        self.declare_parameter('center_y', -3.0)
        self.declare_parameter('radius', 0.7)

        self.declare_parameter('speed', 3.0)
        self.declare_parameter('direction', -1)

        self.declare_parameter('k_r', 3.0)
        self.declare_parameter('k_psi', 0.8)
        self.declare_parameter('k_dr', 0.3)
        self.declare_parameter('d_filter_hz', 7.0)

        self.declare_parameter('v_max', 0.85)
        self.declare_parameter('w_max', 0.75)

        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('odom_timeout_s', 0.3)

        self.declare_parameter('ned_to_enu', True)
        self.declare_parameter('quat_order', 'wxyz')  # 'wxyz' or 'xyzw'

        self.have_odom = False
        self.last_time = None
        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0
        self.e_r_prev = 0.0
        self.e_r_dot_f = 0.0

        odom_topic = self.get_parameter('odom_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(VehicleOdometry, odom_topic, self.odom_cb, odom_qos)
        self.pub = self.create_publisher(Twist, cmd_topic, 5)
        self.timer = self.create_timer(1.0 / rate_hz, self.step)

        self.get_logger().info(f"Sub: {odom_topic}  Pub: {cmd_topic}")

    def odom_cb(self, msg: VehicleOdometry) -> None:
        x = float(msg.position[0])
        y = float(msg.position[1])

        ned_to_enu = bool(self.get_parameter('ned_to_enu').value)
        if ned_to_enu:
            self.px = y
            self.py = x
        else:
            self.px = x
            self.py = y

        q = msg.q
        order = self.get_parameter('quat_order').value
        if order == 'xyzw':
            xq, yq, zq, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        else:
            w, xq, yq, zq = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        yaw = yaw_from_quat_wxyz(w, xq, yq, zq)
        if ned_to_enu:
            yaw = wrap_pi((math.pi / 2.0) - yaw)

        self.yaw = yaw
        self.have_odom = True
        self.last_time = self.get_clock().now()

    def step(self) -> None:
        if not self.have_odom or self.last_time is None:
            self.publish_stop()
            return

        age = (self.get_clock().now() - self.last_time).nanoseconds * 1e-9
        if age > float(self.get_parameter('odom_timeout_s').value):
            self.publish_stop()
            return

        cx = float(self.get_parameter('center_x').value)
        cy = float(self.get_parameter('center_y').value)
        radius = float(self.get_parameter('radius').value)

        v_tan = float(self.get_parameter('speed').value)
        direction = 1 if int(self.get_parameter('direction').value) >= 0 else -1

        k_r = float(self.get_parameter('k_r').value)
        k_psi = float(self.get_parameter('k_psi').value)

        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        # Position bezogen auf Kreiszentrum
        dx = self.px - cx
        dy = self.py - cy

        self.get_logger().info(f"pos=({self.px:.2f}, {self.py:.2f}), yaw={math.degrees(self.yaw):.1f} deg, dx={dx:.2f} dy={dy:.2f}")
        # Abstand zum Kreiszentrum
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            dist = 1e-6
            dx, dy = 1e-6, 0.0
        # Radialer Fehler
        e_r = dist - radius
        self.get_logger().info(f"e_r={e_r:.2f}")
        # Gewünschter Kurswinkel
        phi = math.atan2(dy, dx)
        # Gewünsster Gierwinkel
        yaw_des = phi + direction * (math.pi / 2.0)
        #Sprünge vermeiden
        yaw_des = wrap_pi(yaw_des)
        
        self.get_logger().info(f"yaw_des={math.degrees(yaw_des):.1f} deg")
        # Gierwinkel-Fehler
        e_psi = wrap_pi(yaw_des - self.yaw)
        self.get_logger().info(f"e_psi={math.degrees(e_psi):.1f} deg")
        # Steuerbefehle
        dt = 1.0 / float(self.get_parameter('rate_hz').value)
        e_r_dot = (e_r - self.e_r_prev) / max(1e-3, dt)
        self.e_r_prev = e_r

        d_filter_hz = float(self.get_parameter('d_filter_hz').value)
        alpha = max(0.0, min(1.0, 2.0 * math.pi * d_filter_hz * dt))
        self.e_r_dot_f = (1.0 - alpha) * self.e_r_dot_f + alpha * e_r_dot
        self.get_logger().info(f"e_r_dot={e_r_dot:.2f}, e_r_dot_f={self.e_r_dot_f:.2f}")

        k_dr = float(self.get_parameter('k_dr').value)
        w = k_psi * e_psi + direction * (k_r * e_r + k_dr * self.e_r_dot_f)
        #v = v_tan * max(0.0, (1.0 - abs(e_psi) / (math.pi / 2.0)))
        # statt v = v_tan * max(0.0, (1.0 - abs(e_psi) / (math.pi / 2.0)))
        alpha = 1.3  # größer = stärkeres Abbremsen bei e_psi
        v = v_tan * math.exp(-alpha * abs(e_psi))
        v = max(0.38, v)  # minimale Vorwärtsgeschwindigkeit, z. B. 0.2 m/s
        v = max(-v_max, min(v_max, v))
        w = max(-w_max, min(w_max, w))
        z = 0.0
        if abs(e_r) < 0.1:
            z = 0.4
            
        #w = 0.0
        self.get_logger().info(f"v={v:.2f}, w={w:.1f} rad/s") 
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        cmd.linear.z = z
        self.pub.publish(cmd)

    def publish_stop(self) -> None:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        cmd.linear.z = 0.0
        self.pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = CircleCmdVelPX4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
