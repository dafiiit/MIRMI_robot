#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String


class CmdVelMuxLfi(Node):
    """Mux for BC/LfI data collection.

    Modes:
    - bc: prefer human commands (policy ignored unless human stale and allow_policy_fallback_in_bc=true)
    - lfi: prefer policy commands, human overrides when fresh

    Publishes:
    - /cmd_vel
    - /tool_speed_cmd
    - /intervening (Bool)
    - /mux_source (String: "human"|"policy"|"none")
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_mux_lfi")

        # Topics
        self.declare_parameter("cmd_policy_topic", "/cmd_vel_policy")
        self.declare_parameter("cmd_human_topic", "/cmd_vel_human")
        self.declare_parameter("cmd_out_topic", "/cmd_vel")

        self.declare_parameter("tool_policy_topic", "/tool_speed_policy")
        self.declare_parameter("tool_human_topic", "/tool_speed_human")
        self.declare_parameter("tool_out_topic", "/tool_speed_cmd")

        self.declare_parameter("intervening_topic", "/intervening")
        self.declare_parameter("source_topic", "/mux_source")

        # Behavior
        self.declare_parameter("mode", "lfi")  # "bc" or "lfi"
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("timeout_policy_s", 0.3)
        self.declare_parameter("timeout_human_s", 0.3)
        self.declare_parameter("allow_policy_fallback_in_bc", False)
        self.declare_parameter("use_cmd_linear_z_for_tool", False)

        # Safety clamps
        self.declare_parameter("max_v", 0.8)
        self.declare_parameter("max_w", 2.0)
        self.declare_parameter("max_tool", 1.0)

        self.last_policy_cmd = Twist()
        self.last_human_cmd = Twist()
        self.last_policy_tool = 0.0
        self.last_human_tool = 0.0
        self.t_policy = None
        self.t_human = None

        self.sub_cmd_policy = self.create_subscription(
            Twist, str(self.get_parameter("cmd_policy_topic").value), self.on_cmd_policy, 20
        )
        self.sub_cmd_human = self.create_subscription(
            Twist, str(self.get_parameter("cmd_human_topic").value), self.on_cmd_human, 20
        )

        self.sub_tool_policy = self.create_subscription(
            Float32, str(self.get_parameter("tool_policy_topic").value), self.on_tool_policy, 20
        )
        self.sub_tool_human = self.create_subscription(
            Float32, str(self.get_parameter("tool_human_topic").value), self.on_tool_human, 20
        )

        self.pub_cmd = self.create_publisher(Twist, str(self.get_parameter("cmd_out_topic").value), 20)
        self.pub_tool = self.create_publisher(Float32, str(self.get_parameter("tool_out_topic").value), 20)
        self.pub_intervening = self.create_publisher(Bool, str(self.get_parameter("intervening_topic").value), 20)
        self.pub_source = self.create_publisher(String, str(self.get_parameter("source_topic").value), 20)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self.step)

        self.get_logger().info("cmd_vel_mux_lfi started")

    def now(self):
        return self.get_clock().now()

    def is_fresh(self, t, timeout_s: float) -> bool:
        if t is None:
            return False
        age_s = (self.now() - t).nanoseconds * 1e-9
        return age_s <= timeout_s

    def on_cmd_policy(self, msg: Twist) -> None:
        self.last_policy_cmd = msg
        if bool(self.get_parameter("use_cmd_linear_z_for_tool").value):
            self.last_policy_tool = float(msg.linear.z)
        self.t_policy = self.now()

    def on_cmd_human(self, msg: Twist) -> None:
        self.last_human_cmd = msg
        if bool(self.get_parameter("use_cmd_linear_z_for_tool").value):
            self.last_human_tool = float(msg.linear.z)
        self.t_human = self.now()

    def on_tool_policy(self, msg: Float32) -> None:
        self.last_policy_tool = float(msg.data)
        self.t_policy = self.now()

    def on_tool_human(self, msg: Float32) -> None:
        self.last_human_tool = float(msg.data)
        self.t_human = self.now()

    def clamp_cmd(self, cmd: Twist) -> Twist:
        out = Twist()
        max_v = float(self.get_parameter("max_v").value)
        max_w = float(self.get_parameter("max_w").value)
        out.linear.x = float(max(-max_v, min(max_v, cmd.linear.x)))
        out.angular.z = float(max(-max_w, min(max_w, cmd.angular.z)))
        return out

    def clamp_tool(self, s: float) -> float:
        max_tool = float(self.get_parameter("max_tool").value)
        return float(max(0.0, min(max_tool, s)))

    def step(self) -> None:
        mode = str(self.get_parameter("mode").value).strip().lower()
        timeout_policy_s = float(self.get_parameter("timeout_policy_s").value)
        timeout_human_s = float(self.get_parameter("timeout_human_s").value)

        human_fresh = self.is_fresh(self.t_human, timeout_human_s)
        policy_fresh = self.is_fresh(self.t_policy, timeout_policy_s)

        source = "none"
        intervening = False
        cmd = Twist()
        tool = 0.0

        if mode == "bc":
            if human_fresh:
                source = "human"
                intervening = True
                cmd = self.last_human_cmd
                tool = self.last_human_tool
            elif bool(self.get_parameter("allow_policy_fallback_in_bc").value) and policy_fresh:
                source = "policy"
                intervening = False
                cmd = self.last_policy_cmd
                tool = self.last_policy_tool
        else:
            # lfi mode: policy by default, human overrides when fresh
            if human_fresh:
                source = "human"
                intervening = True
                cmd = self.last_human_cmd
                tool = self.last_human_tool
            elif policy_fresh:
                source = "policy"
                intervening = False
                cmd = self.last_policy_cmd
                tool = self.last_policy_tool

        cmd = self.clamp_cmd(cmd)
        tool = self.clamp_tool(tool)

        if bool(self.get_parameter("use_cmd_linear_z_for_tool").value):
            cmd.linear.z = float(tool)

        self.pub_cmd.publish(cmd)
        self.pub_tool.publish(Float32(data=tool))
        self.pub_intervening.publish(Bool(data=intervening))
        self.pub_source.publish(String(data=source))


def main() -> None:
    rclpy.init()
    node = CmdVelMuxLfi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
