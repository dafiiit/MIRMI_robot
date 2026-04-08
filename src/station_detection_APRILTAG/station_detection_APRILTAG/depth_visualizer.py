import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
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
        
        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        self.sub = self.create_subscription(
            Image,
            in_topic,
            self.callback,
            10
        )
        
        # Publish regular image (which will be compressed by simple_launch or downstream)
        # But to be safe and efficient, let's explicitly publish compressed here too?
        # No, standard image_transport is better, but this is a python node.
        # rclpy doesn't have image_transport built-in the same way.
        # So I will publish BOTH raw (for local) and compressed (for internal use) manually
        # OR just publish raw and rely on ros2 run image_transport republish if needed?
        # BETTER: Just use cv2.imencode to publish CompressedImage directly. It's lighter.
        
        self.pub_raw = self.create_publisher(Image, out_topic + '/raw', 10)
        self.pub_compressed = self.create_publisher(CompressedImage, out_topic + '/compressed', 10)
        
        self.get_logger().info(f"DepthVisualizer converting {in_topic} -> {out_topic}/compressed")

    def callback(self, msg):
        try:
            # Convert ROS Image to OpenCV serialization
            # depth 16UC1
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
            # Normalize to 0-255 for visualization
            # D435i default scale is 1mm. 
            # We want to see reasonable range, e.g., 0 to 3 meters (3000mm)
            # Clip at 3000 ? or just normalize min-max?
            # Fixed range is better for video stability. 0-4m is good.
            
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
