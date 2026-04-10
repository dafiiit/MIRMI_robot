"""
Depth Visualizer Node
=====================
This node processes raw 16-bit depth images and converts them into a 
human-readable 8-bit colorized format (JET colormap) for easy visualization.

What it does:
- Subscribes to a depth image stream (default: /camera/camera/aligned_depth_to_color/image_raw).
- Clips values to a 0-4m range.
- Applies a JET colormap (near = red/yellow, far = blue).
- Publishes both a Raw (BGR8) and Compressed (JPEG) image.

How to start manually:
  ros2 run station_detection_APRILTAG depth_visualizer

Optional Parameters:
  - input_topic (string): The topic to subscribe to.
  - output_topic (string): The base name for the output topics.
  - auto_enable_depth (bool, default True): If True, attempts to call the camera 
    node services to enable depth and aligned_depth if they are disabled.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

import cv2
import numpy as np

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('output_topic', '/camera/depth_visualization')
        self.declare_parameter('auto_enable_depth', True)
        
        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable_depth').get_parameter_value().bool_value
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            Image,
            in_topic,
            self.callback,
            qos
        )
        
        self.pub_raw = self.create_publisher(Image, out_topic + '/raw', qos)
        self.pub_compressed = self.create_publisher(CompressedImage, out_topic + '/compressed', qos)

        
        self.get_logger().info(f"DepthVisualizer converting {in_topic} -> {out_topic}/compressed")

        if self.auto_enable:
            self.enable_camera_depth()

    def enable_camera_depth(self):
        self.get_logger().info("Attempting to auto-enable depth on Realsense node...")
        # Create client for camera parameter service
        client = self.create_client(SetParameters, '/camera/camera/set_parameters')
        
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Camera parameter service not available (is the camera node running?). Skipping auto-enable.")
            return

        # Prepare parameters
        p_enable_depth = Parameter()
        p_enable_depth.name = 'enable_depth'
        p_enable_depth.value.type = ParameterType.PARAMETER_BOOL
        p_enable_depth.value.bool_value = True

        p_align_depth = Parameter()
        p_align_depth.name = 'align_depth.enable'
        p_align_depth.value.type = ParameterType.PARAMETER_BOOL
        p_align_depth.value.bool_value = True

        req = SetParameters.Request()
        req.parameters = [p_enable_depth, p_align_depth]

        # Call service asynchronously
        future = client.call_async(req)
        # We don't wait for the future here to avoid blocking spin, 
        # but we can add a callback for logging
        future.add_done_callback(self.enable_callback)

    def enable_callback(self, future):
        try:
            response = future.result()
            for res in response.results:
                if not res.successful:
                    self.get_logger().error(f"Failed to set camera parameter: {res.reason}")
            self.get_logger().info("Successfully requested depth streams from camera.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def callback(self, msg):
        try:
            # Convert ROS Image to OpenCV serialization
            # depth 16UC1
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Normalize to 0-4000mm range for visualization
            depth_clipped = np.clip(cv_image, 0, 4000)
            depth_normalized = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply ColorMap (JET is standard for depth)
            depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # Publish Raw (RGB8)
            out_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")
            out_msg.header = msg.header
            self.pub_raw.publish(out_msg)
            
            # Publish Compressed (JPEG)
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            success, encoded_image = cv2.imencode('.jpg', depth_color, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if success:
                compressed_msg.data = encoded_image.tobytes()
                self.pub_compressed.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
