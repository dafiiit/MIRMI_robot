import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.declare_parameter('topic', 'Position/GPS')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('latitude', 0.0)
        self.declare_parameter('longitude', 0.0)
        self.declare_parameter('altitude', 0.0)
        self.declare_parameter('frame_id', 'gps')

        topic = self.get_parameter('topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.publisher = self.create_publisher(NavSatFix, topic, 10)
        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(period, self.publish_fix)

    def publish_fix(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        msg.latitude = self.get_parameter('latitude').get_parameter_value().double_value
        msg.longitude = self.get_parameter('longitude').get_parameter_value().double_value
        msg.altitude = self.get_parameter('altitude').get_parameter_value().double_value

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
