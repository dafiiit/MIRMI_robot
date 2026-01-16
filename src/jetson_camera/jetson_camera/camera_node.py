import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('jetson_camera_node')
        self.bridge = CvBridge()

        # 1. QoS für WLAN (Compressed): Schnell, darf verwerfen (Best Effort)
        qos_wlan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.compressed_pub_ = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', qos_wlan)
        
        # 2. QoS für LOKAL (Raw): Zuverlässig (Reliable), damit apriltag_ros nicht meckert
        qos_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.raw_pub_ = self.create_publisher(Image, '/camera/image_raw', qos_local)
        
        # 3. Camera Info Publisher (Required by AprilTag)
        self.info_pub_ = self.create_publisher(CameraInfo, '/camera/camera_info', qos_local)

        self.timer = self.create_timer(1.0/5.0, self.timer_callback) # 5 Hz
        
        # Pipeline bleibt gleich
        self.pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)5/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink drop=true max-buffers=1"
        )
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            now = self.get_clock().now().to_msg()
            frame_id = "camera_link"

            # Raw (Lokal -> Reliable)
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header.stamp = now
            raw_msg.header.frame_id = frame_id
            self.raw_pub_.publish(raw_msg)

            # Camera Info (User provided calibration)
            info_msg = CameraInfo()
            info_msg.header.stamp = now
            info_msg.header.frame_id = frame_id
            info_msg.width = 640
            info_msg.height = 480
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.k = [554.256, 0.0, 320.5,
                          0.0, 554.256, 240.5,
                          0.0, 0.0, 1.0]
            info_msg.r = [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
            info_msg.p = [554.256, 0.0, 320.5, 0.0,
                          0.0, 554.256, 240.5, 0.0,
                          0.0, 0.0, 1.0, 0.0]
            
            self.info_pub_.publish(info_msg)

            # Compressed (WLAN -> Best Effort)
            msg = CompressedImage()
            msg.header.stamp = now
            msg.header.frame_id = frame_id
            msg.format = "jpeg"
            success, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if success:
                msg.data = encoded_image.tobytes()
                self.compressed_pub_.publish(msg)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()