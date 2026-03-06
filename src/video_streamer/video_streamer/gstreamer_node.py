import subprocess
import rclpy
from rclpy.node import Node

class GStreamerNode(Node):
    def __init__(self):
        super().__init__('gstreamer_node')

        # Declare parameters with defaults
        self.declare_parameter('width', 960)
        self.declare_parameter('height', 480)
        self.declare_parameter('framerate', 24)
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('host_ip', '10.157.175.72')
        self.declare_parameter('port', 5000)

        # Read parameters
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        framerate = self.get_parameter('framerate').value
        bitrate = self.get_parameter('bitrate').value
        host_ip = self.get_parameter('host_ip').value
        port = self.get_parameter('port').value

        # Build the pipeline dynamically
        pipeline = (
            'gst-launch-1.0 -v '
            f'nvarguscamerasrc sensor-id=0 ! '
            f'"video/x-raw(memory:NVMM), width={width}, height={height}, framerate={framerate}/1" ! '
            'queue ! nvvidconv ! '
            f'"video/x-raw(memory:NVMM), format=NV12" ! '
            f'queue ! nvv4l2h264enc bitrate={bitrate} profile=2 preset-level=1 '
            'iframeinterval=30 idrinterval=30 insert-sps-pps=1 ! '
            'queue ! rtph264pay config-interval=1 pt=96 ! '
            f'queue ! udpsink host={host_ip} port={port}'
        )

        self.get_logger().info("Starting GStreamer pipeline with parameters:")
        self.get_logger().info(f"  width={width}, height={height}, framerate={framerate}")
        self.get_logger().info(f"  bitrate={bitrate}, host_ip={host_ip}, port={port}")
        self.get_logger().info(f"Pipeline:\n{pipeline}")

        # Launch the pipeline
        self.process = subprocess.Popen(pipeline, shell=True)

    def destroy_node(self):
        self.get_logger().info("Stopping GStreamer pipeline...")
        self.process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

