#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class CmdVelMuxSimple(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_mux_simple')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('joy_cmd_topic', '/cmd_vel_joy')
        self.declare_parameter('auto_cmd_topic', '/cmd_vel_circle')
        self.declare_parameter('out_cmd_topic', '/cmd_vel')

        self.declare_parameter('enable_button_index', 4)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('timeout_s', 0.2)

        self.enable = False
        self.last_joy_cmd = Twist()
        self.last_auto_cmd = Twist()
        self.t_joy = None
        self.t_auto = None

        joy_topic = self.get_parameter('joy_topic').value
        joy_cmd_topic = self.get_parameter('joy_cmd_topic').value
        auto_cmd_topic = self.get_parameter('auto_cmd_topic').value
        out_cmd_topic = self.get_parameter('out_cmd_topic').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.sub_joy = self.create_subscription(Joy, joy_topic, self.on_joy, 10)
        self.sub_cmd_joy = self.create_subscription(Twist, joy_cmd_topic, self.on_cmd_joy, 10)
        self.sub_cmd_auto = self.create_subscription(Twist, auto_cmd_topic, self.on_cmd_auto, 10)
        self.pub = self.create_publisher(Twist, out_cmd_topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.step)

        self.get_logger().info(
            f"Mux: joy={joy_cmd_topic}, auto={auto_cmd_topic} -> out={out_cmd_topic}"
        )

    def on_joy(self, msg: Joy) -> None:
        idx = int(self.get_parameter('enable_button_index').value)
        pressed = idx < len(msg.buttons) and msg.buttons[idx] == 1
        self.enable = pressed

    def on_cmd_joy(self, msg: Twist) -> None:
        self.last_joy_cmd = msg
        self.t_joy = self.get_clock().now()

    def on_cmd_auto(self, msg: Twist) -> None:
        self.last_auto_cmd = msg
        self.t_auto = self.get_clock().now()

    def fresh(self, t) -> bool:
        if t is None:
            return False
        age = (self.get_clock().now() - t).nanoseconds * 1e-9
        return age <= float(self.get_parameter('timeout_s').value)

    def step(self) -> None:
        if self.enable:
            cmd = self.last_auto_cmd if self.fresh(self.t_auto) else Twist()
        else:
            cmd = self.last_joy_cmd if self.fresh(self.t_joy) else Twist()
        self.pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = CmdVelMuxSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
