
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2

class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        self.bridge = CvBridge()

        # Parameter for Device ID
        self.declare_parameter('device_id', 1) # Default to 1 (assuming 0 is the board camera)
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value

        # 1. QoS for WLAN (Compressed): Fast, Best Effort
        qos_wlan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.compressed_pub_ = self.create_publisher(CompressedImage, '/usb_camera/image_raw/compressed', qos_wlan)
        
        # 2. QoS for LOCAL (Raw): Reliable (for apriltag_ros)
        qos_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.raw_pub_ = self.create_publisher(Image, '/usb_camera/image_raw', qos_local)
        
        # 3. Camera Info Publisher
        self.info_pub_ = self.create_publisher(CameraInfo, '/usb_camera/camera_info', qos_local)

        self.get_logger().info(f'Opening USB Camera at device index {self.device_id}...')
        self.cap = cv2.VideoCapture(self.device_id)
        
        # Set resolution to 640x480 to match the other camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video device {self.device_id}!')

        self.timer = self.create_timer(1.0/10.0, self.timer_callback) # 10 Hz (USB cams are often 30, but let's be safe)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            now = self.get_clock().now().to_msg()
            frame_id = "usb_camera_link"

            # Raw
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header.stamp = now
            raw_msg.header.frame_id = frame_id
            self.raw_pub_.publish(raw_msg)

            # Camera Info (Dummy calibration for now, as requested)
            info_msg = CameraInfo()
            info_msg.header.stamp = now
            info_msg.header.frame_id = frame_id
            info_msg.width = 640
            info_msg.height = 480
            info_msg.distortion_model = "plumb_bob"
            # Using same k/p as other camera as a starting point is dangerous but 
            # user asked for "separate camera info... because it is calibrated differently... 
            # but write it so it works now, I calibrate later".
            # I will put Identity matrix or generic values to avoid wildly wrong projections if possible,
            # but AprilTag needs reasonable focal length to estimate distance.
            # I'll stick to the values from the other camera but maybe comment that they need update.
            # Actually, for "works now", reusing the other camera's intrinsics is the "least bad" guess 
            # if the lens is somewhat similar (webcams are wide), but probably wildly off.
            # However, typically webcams have larger FOV.
            # I'll modify K a bit to be more generic fx=fy=500 for 640x480 is typical.
            
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.k = [500.0, 0.0, 320.0,
                          0.0, 500.0, 240.0,
                          0.0, 0.0, 1.0]
            info_msg.r = [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
            info_msg.p = [500.0, 0.0, 320.0, 0.0,
                          0.0, 500.0, 240.0, 0.0,
                          0.0, 0.0, 1.0, 0.0]
            
            self.info_pub_.publish(info_msg)

            # Compressed
            msg = CompressedImage()
            msg.header.stamp = now
            msg.header.frame_id = frame_id
            msg.format = "jpeg"
            success, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if success:
                msg.data = encoded_image.tobytes()
                self.compressed_pub_.publish(msg)
        else:
            self.get_logger().warn('Failed to capture frame from USB camera')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
