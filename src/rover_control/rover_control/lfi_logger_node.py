#!/usr/bin/env python3
import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


@dataclass
class Twist3:
    v: float = 0.0
    w: float = 0.0
    s: float = 0.0


class LFILoggerNode(Node):
    """Log BC/LfI data into compressed NPZ files."""

    def __init__(self) -> None:
        super().__init__("lfi_logger_node")

        self.declare_parameter("out_dir", "/tmp/lfi_logs")
        self.declare_parameter("session_name", "session")
        self.declare_parameter("sample_hz", 15.0)
        self.declare_parameter("max_seconds", 0.0)
        self.declare_parameter("clip_lidar_max", 10.0)
        self.declare_parameter("require_pole_conf_min", 0.0)

        self.declare_parameter("lidar_topic", "/lidar_polar")
        self.declare_parameter("pole_topic", "/pole_estimate")
        self.declare_parameter("pole_conf_topic", "/pole_confidence")
        self.declare_parameter("pole_diam_topic", "/pole_diameter")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter("cmd_policy_topic", "/cmd_vel_policy")
        self.declare_parameter("cmd_human_topic", "/cmd_vel_human")
        self.declare_parameter("cmd_exec_topic", "/cmd_vel")
        self.declare_parameter("tool_policy_topic", "/tool_speed_policy")
        self.declare_parameter("tool_human_topic", "/tool_speed_human")
        self.declare_parameter("tool_exec_topic", "/tool_speed_cmd")
        self.declare_parameter("intervening_topic", "/intervening")

        self.latest_scan: Optional[LaserScan] = None
        self.latest_pole: Optional[PointStamped] = None
        self.latest_pole_conf: float = 0.0
        self.latest_pole_diam: float = 0.0
        self.latest_v: float = 0.0
        self.latest_w: float = 0.0
        self.latest_policy = Twist3()
        self.latest_human = Twist3()
        self.latest_exec = Twist3()
        self.latest_intervening: int = 0

        self.records = []
        self.t0 = time.time()

        self.create_subscription(LaserScan, str(self.get_parameter("lidar_topic").value), self.on_scan, 10)
        self.create_subscription(PointStamped, str(self.get_parameter("pole_topic").value), self.on_pole, 10)
        self.create_subscription(Float32, str(self.get_parameter("pole_conf_topic").value), self.on_pole_conf, 10)
        self.create_subscription(Float32, str(self.get_parameter("pole_diam_topic").value), self.on_pole_diam, 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 20)

        self.create_subscription(Twist, str(self.get_parameter("cmd_policy_topic").value), self.on_cmd_policy, 20)
        self.create_subscription(Twist, str(self.get_parameter("cmd_human_topic").value), self.on_cmd_human, 20)
        self.create_subscription(Twist, str(self.get_parameter("cmd_exec_topic").value), self.on_cmd_exec, 20)

        self.create_subscription(Float32, str(self.get_parameter("tool_policy_topic").value), self.on_tool_policy, 20)
        self.create_subscription(Float32, str(self.get_parameter("tool_human_topic").value), self.on_tool_human, 20)
        self.create_subscription(Float32, str(self.get_parameter("tool_exec_topic").value), self.on_tool_exec, 20)

        self.create_subscription(Bool, str(self.get_parameter("intervening_topic").value), self.on_intervening, 50)

        hz = float(self.get_parameter("sample_hz").value)
        self.timer = self.create_timer(1.0 / max(hz, 1e-3), self.tick)

        self.get_logger().info("LFILoggerNode started")

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_pole(self, msg: PointStamped):
        self.latest_pole = msg

    def on_pole_conf(self, msg: Float32):
        self.latest_pole_conf = float(msg.data)

    def on_pole_diam(self, msg: Float32):
        self.latest_pole_diam = float(msg.data)

    def on_odom(self, msg: Odometry):
        self.latest_v = float(msg.twist.twist.linear.x)
        self.latest_w = float(msg.twist.twist.angular.z)

    def on_cmd_policy(self, msg: Twist):
        self.latest_policy.v = float(msg.linear.x)
        self.latest_policy.w = float(msg.angular.z)

    def on_cmd_human(self, msg: Twist):
        self.latest_human.v = float(msg.linear.x)
        self.latest_human.w = float(msg.angular.z)

    def on_cmd_exec(self, msg: Twist):
        self.latest_exec.v = float(msg.linear.x)
        self.latest_exec.w = float(msg.angular.z)

    def on_tool_policy(self, msg: Float32):
        self.latest_policy.s = float(msg.data)

    def on_tool_human(self, msg: Float32):
        self.latest_human.s = float(msg.data)

    def on_tool_exec(self, msg: Float32):
        self.latest_exec.s = float(msg.data)

    def on_intervening(self, msg: Bool):
        self.latest_intervening = 1 if msg.data else 0

    def tick(self):
        max_seconds = float(self.get_parameter("max_seconds").value)
        if max_seconds > 0.0 and (time.time() - self.t0) > max_seconds:
            raise KeyboardInterrupt

        if self.latest_scan is None or self.latest_pole is None:
            return

        conf_min = float(self.get_parameter("require_pole_conf_min").value)
        if self.latest_pole_conf < conf_min:
            return

        clip_lidar_max = float(self.get_parameter("clip_lidar_max").value)
        ranges = np.asarray(self.latest_scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=clip_lidar_max, posinf=clip_lidar_max, neginf=0.0)
        ranges = np.clip(ranges, 0.0, clip_lidar_max)

        extra = np.array(
            [
                float(self.latest_pole.point.x),
                float(self.latest_pole.point.y),
                float(self.latest_pole_conf),
                float(self.latest_pole_diam),
                float(self.latest_v),
                float(self.latest_w),
            ],
            dtype=np.float32,
        )

        a_policy = np.array([self.latest_policy.v, self.latest_policy.w, self.latest_policy.s], dtype=np.float32)
        a_human = np.array([self.latest_human.v, self.latest_human.w, self.latest_human.s], dtype=np.float32)
        a_exec = np.array([self.latest_exec.v, self.latest_exec.w, self.latest_exec.s], dtype=np.float32)

        t = np.float32(time.time() - self.t0)
        intervening = np.uint8(self.latest_intervening)

        self.records.append((t, ranges, extra, a_policy, a_human, a_exec, intervening))

    def save(self):
        if not self.records:
            self.get_logger().warn("No records captured; nothing saved")
            return

        out_dir = str(self.get_parameter("out_dir").value)
        session_name = str(self.get_parameter("session_name").value)
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(out_dir, f"{session_name}_{ts}.npz")

        t = np.stack([r[0] for r in self.records]).astype(np.float32)
        lidar = np.stack([r[1] for r in self.records]).astype(np.float32)
        extra = np.stack([r[2] for r in self.records]).astype(np.float32)
        a_policy = np.stack([r[3] for r in self.records]).astype(np.float32)
        a_human = np.stack([r[4] for r in self.records]).astype(np.float32)
        a_exec = np.stack([r[5] for r in self.records]).astype(np.float32)
        intervening = np.stack([r[6] for r in self.records]).astype(np.uint8)

        np.savez_compressed(
            out_path,
            t=t,
            lidar=lidar,
            extra=extra,
            a_policy=a_policy,
            a_human=a_human,
            a_exec=a_exec,
            intervening=intervening,
        )

        self.get_logger().info(f"Saved {len(self.records)} samples to {out_path}")


def main() -> None:
    rclpy.init()
    node = LFILoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
