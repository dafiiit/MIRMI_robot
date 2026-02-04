import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class IRTrackerNode(Node):
    def __init__(self):
        super().__init__('ir_tracker_node')

        # Parameters
        self.declare_parameter('threshold', 50)
        self.threshold_val = self.get_parameter('threshold').value
        
        # Subscribers
        self.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', self.infra_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/infra1/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/ir_led/pose', 10)
        self.debug_pub = self.create_publisher(Image, '/ir_led/debug_view', 10)

        self.bridge = CvBridge()
        self.last_frame = None
        self.depth_frame = None
        self.camera_intrinsics = None

        self.get_logger().info('IR Tracker Node Started')

    def info_callback(self, msg):
        self.camera_intrinsics = msg
        # Once we have intrinsics, we might not need to keep subscribing, but it's fine.

    def depth_callback(self, msg):
        try:
            # D435i depth is 16-bit integer (mm)
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def infra_callback(self, msg):
        try:
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Infra callback error: {e}')
            return

        if self.last_frame is None:
            self.last_frame = current_frame
            return

        # 1. Frame Difference to remove static background (projector pattern)
        diff = cv2.absdiff(current_frame, self.last_frame)
        self.last_frame = current_frame

        # 2. Thresholding
        _, mask = cv2.threshold(diff, self.threshold_val, 255, cv2.THRESH_BINARY)

        # 3. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Assume the largest moving blob is the LED
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # 4. Get 3D Position
                if self.depth_frame is not None and self.camera_intrinsics is not None:
                    # Check bounds
                    h, w = self.depth_frame.shape
                    if 0 <= cY < h and 0 <= cX < w:
                        depth_val = self.depth_frame[cY, cX] 
                        
                        # Depth is typically in mm for Realsense, convert to meters
                        depth_m = depth_val * 0.001

                        if depth_m > 0:
                            point_3d = self.project_pixel_to_3d(cX, cY, depth_m)
                            self.publish_pose(point_3d, msg.header)

        # Debug View
        if self.debug_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def project_pixel_to_3d(self, u, v, z):
        # Use camera intrinsics to project
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        if self.camera_intrinsics is None:
            return None
        
        fx = self.camera_intrinsics.k[0]
        fy = self.camera_intrinsics.k[4]
        cx = self.camera_intrinsics.k[2]
        cy = self.camera_intrinsics.k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        # z is z

        return Point(x=x, y=y, z=z)

    def publish_pose(self, point, header):
        if point is None:
            return
            
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position = point
        pose_msg.pose.orientation.w = 1.0 # Identity quaternion
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IRTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
