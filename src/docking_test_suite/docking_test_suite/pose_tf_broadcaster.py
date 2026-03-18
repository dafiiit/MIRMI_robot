"""Broadcast TF from PoseStamped topics.

This is intended to provide TF links even when the upstream system publishes
PoseStamped but not TF (or when TF is too noisy/absent).
"""

from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseTfBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_tf_broadcaster')

        self.declare_parameter('parent_frame', 'vicon/world')

        # Robot pose -> vicon/Robot_1/Robot_1
        self.declare_parameter('robot_pose_topic', '/vicon/Robot_1/Robot_1/pose')
        self.declare_parameter('robot_child_frame', 'vicon/Robot_1/Robot_1')

        # Target pose -> vicon/Tag_0/Tag_0
        self.declare_parameter('target_pose_topic', '/vicon/Tag_0/Tag_0/pose')
        self.declare_parameter('target_child_frame', 'vicon/Tag_0/Tag_0')

        self._parent_frame = self.get_parameter('parent_frame').value
        self._robot_topic = self.get_parameter('robot_pose_topic').value
        self._robot_child = self.get_parameter('robot_child_frame').value
        self._target_topic = self.get_parameter('target_pose_topic').value
        self._target_child = self.get_parameter('target_child_frame').value

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(PoseStamped, self._robot_topic, self._robot_cb, 10)
        self.create_subscription(PoseStamped, self._target_topic, self._target_cb, 10)

        self.get_logger().info(
            f'Broadcasting TF from PoseStamped. parent_frame="{self._parent_frame}" '
            f'robot: {self._robot_topic} -> {self._robot_child} '
            f'target: {self._target_topic} -> {self._target_child}'
        )

    def _send_pose_as_tf(self, msg: PoseStamped, child_frame: str):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        t.header.frame_id = self._parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def _robot_cb(self, msg: PoseStamped):
        self._send_pose_as_tf(msg, self._robot_child)

    def _target_cb(self, msg: PoseStamped):
        self._send_pose_as_tf(msg, self._target_child)


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = PoseTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

