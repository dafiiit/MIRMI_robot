import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class PoleMarker(Node):
    def __init__(self):
        super().__init__("pole_marker")

        self.sub_pos = self.create_subscription(
            PointStamped, "/pole_estimate", self.on_pos, 10
        )
        self.sub_rad = self.create_subscription(
            Float32, "/pole_radius", self.on_radius, 10
        )
        self.sub_conf = self.create_subscription(
            Float32, "/pole_confidence", self.on_confidence, 10
        )

        self.pub_markers = self.create_publisher(MarkerArray, "/tree_markers", 10)

        self.latest_stamp = None
        self.current_detections = []   # <--- store multiple trees

        self.latest_rad = None
        self.latest_conf = None

    def on_pos(self, msg):
        stamp = msg.header.stamp

        # New frame → publish previous frame
        if self.latest_stamp is not None and stamp != self.latest_stamp:
            self.publish_frame()
            self.current_detections = []

        self.latest_stamp = stamp

        # Store detection (position only for now)
        self.current_detections.append({
            "pos": msg,
            "rad": self.latest_rad,
            "conf": self.latest_conf
        })

    def on_radius(self, msg):
        self.latest_rad = msg.data

    def on_confidence(self, msg):
        self.latest_conf = msg.data

    def publish_frame(self):
        arr = MarkerArray()

        for i, det in enumerate(self.current_detections):
            pos = det["pos"]
            rad = det["rad"]
            conf = det["conf"]

            if pos is None or rad is None or conf is None:
                continue

            marker = Marker()
            marker.header = pos.header
            marker.ns = "trees"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = pos.point.x
            marker.pose.position.y = pos.point.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = rad
            marker.scale.y = rad
            marker.scale.z = 1.0

            marker.color.r = 1.0 - conf
            marker.color.g = conf
            marker.color.b = 0.0
            marker.color.a = 0.9

            arr.markers.append(marker)

        self.pub_markers.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = PoleMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()