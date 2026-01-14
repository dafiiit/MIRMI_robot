import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import numpy as np

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('jetson_camera_node')
        
        # --- WICHTIG: QoS Settings gegen Latenz ---
        # Depth=1: Nur das allerneueste Bild behalten (Buffer = 1)
        # BEST_EFFORT: Pakete dürfen verloren gehen (UDP-Style), verhindert Stau
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher für KOMPRIMIERTE Bilder (spart 95% Bandbreite!)
        self.image_publisher_ = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', qos_profile)
        self.info_publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile)
        
        self.timer = self.create_timer(1.0/8.0, self.timer_callback)
        
        # Änderungen:
        # 1. framerate=(fraction)5/1  -> Hardware macht direkt nur 5 FPS
        # 2. appsink drop=true max-buffers=1 -> Zwingt GStreamer, alte Bilder sofort zu löschen
        self.pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)5/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink drop=true max-buffers=1"
        )
        
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Konnte GStreamer Pipeline nicht öffnen!')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            now = self.get_clock().now().to_msg()
            frame_id = "camera_link"

            # --- Komprimierung (Der eigentliche Fix) ---
            msg = CompressedImage()
            msg.header.stamp = now
            msg.header.frame_id = frame_id
            msg.format = "jpeg"
            # JPEG Komprimierung mit 50% Qualität (schnell & klein)
            success, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if success:
                msg.data = encoded_image.tobytes()
                self.image_publisher_.publish(msg)

            # CameraInfo (optional, falls benötigt)
            # ... (hier gekürzt für Übersichtlichkeit, Info ist meist klein genug)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()