import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2


class JetsonCameraStreamNode(Node):
    def __init__(self):
        super().__init__('jetson_camera_stream_node')

        self.declare_parameter('target_fps', 30.0)
        self.declare_parameter('sensor_fps', 30)
        self.declare_parameter('capture_width', 640)
        self.declare_parameter('capture_height', 480)
        self.declare_parameter('jpeg_quality', 35)
        self.declare_parameter('flip_method', 0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('topic', '/camera/image_raw/compressed')

        self.target_fps = self.get_parameter('target_fps').get_parameter_value().double_value
        self.sensor_fps = self.get_parameter('sensor_fps').get_parameter_value().integer_value
        self.capture_width = self.get_parameter('capture_width').get_parameter_value().integer_value
        self.capture_height = self.get_parameter('capture_height').get_parameter_value().integer_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.flip_method = self.get_parameter('flip_method').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        qos_stream = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.compressed_pub_ = self.create_publisher(CompressedImage, self.topic, qos_stream)

        timer_period = 1.0 / max(self.target_fps, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            f"video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction){self.sensor_fps}/1 ! "
            f"nvvidconv flip-method={self.flip_method} ! "
            f"video/x-raw, width=(int){self.capture_width}, height=(int){self.capture_height}, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink sync=false drop=true max-buffers=1"
        )
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open CSI camera with gstreamer pipeline')

        self.get_logger().info(
            f'Stream node started: {self.capture_width}x{self.capture_height} '
            f'target_fps={self.target_fps} sensor_fps={self.sensor_fps} '
            f'jpeg_quality={self.jpeg_quality} topic={self.topic}'
        )

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        now = self.get_clock().now().to_msg()

        msg = CompressedImage()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        success, encoded_image = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if success:
            msg.data = encoded_image.tobytes()
            self.compressed_pub_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraStreamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
