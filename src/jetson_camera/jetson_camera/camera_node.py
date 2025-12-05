import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class JetsonCameraNode(Node):
    def __init__(self):
        super().__init__('jetson_camera_node')
        
        # Publisher erstellen
        self.image_publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        self.timer = self.create_timer(1.0/30.0, self.timer_callback) # 30 FPS
        self.bridge = CvBridge()
        
        # --- Kalibrierungsdaten (Deine Werte) ---
        self.camera_name = "jetson_camera"
        self.width = 1280
        self.height = 720

        self.d = [-0.2641643194597187, 0.05147908049721637, 0.0014894888957842174, -0.0008911263195618807, 0.0]
        self.k = [429.30515015582205, 0.0, 646.9219832071054, 
            0.0, 428.0068528105944, 371.0204795998143, 
            0.0, 0.0, 1.0]
        self.r = [1.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 
            0.0, 0.0, 1.0]
        self.p = [285.8752136230469, 0.0, 644.1046707392161, 0.0, 
            0.0, 320.3103332519531, 383.15896688028806, 0.0, 
            0.0, 0.0, 1.0, 0.0]
    
        # GStreamer Pipeline
        self.pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
        )
        
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('Konnte GStreamer Pipeline nicht öffnen!')
        else:
            self.get_logger().info('Kamera gestartet!')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Zeitstempel holen (MUSS für beide Nachrichten gleich sein)
            now = self.get_clock().now().to_msg()
            frame_id = "camera_link"

            # 1. Image Nachricht bauen
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = frame_id
            
            # 2. CameraInfo Nachricht bauen
            info_msg = CameraInfo()
            info_msg.header.stamp = now
            info_msg.header.frame_id = frame_id
            info_msg.height = self.height
            info_msg.width = self.width
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = self.d
            info_msg.k = self.k
            info_msg.r = self.r
            info_msg.p = self.p

            # Beides publishen
            self.image_publisher_.publish(img_msg)
            self.info_publisher_.publish(info_msg)

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