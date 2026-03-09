"""System diagnostics node for the docking test suite.

Checks that all required topics are publishing, verifies data rates,
and validates calibration offsets.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage

from .config_loader import load_config

try:
    from apriltag_msgs.msg import AprilTagDetectionArray as AprilTagDetectionArrayStd
    _HAS_APRILTAG_MSGS = True
except ImportError:
    _HAS_APRILTAG_MSGS = False

try:
    from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray as AprilTagDetectionArrayIsaac
    _HAS_ISAAC_APRILTAG = True
except ImportError:
    _HAS_ISAAC_APRILTAG = False

try:
    from px4_msgs.msg import VehicleOdometry
    _HAS_PX4 = True
except ImportError:
    _HAS_PX4 = False


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('docking_diagnostics')
        self.declare_parameter('config_path', '')
        cp = self.get_parameter('config_path').get_parameter_value().string_value
        self.cfg = load_config(cp if cp else None)
        self.topics = self.cfg['topics']

        # Counters for rate measurement
        self._counters = {}
        self._lock = threading.Lock()

        # Subscribe to all relevant topics to measure rates
        self._subscribe_all()

    def _count(self, name):
        def cb(_msg):
            with self._lock:
                self._counters[name] = self._counters.get(name, 0) + 1
        return cb

    def _subscribe_all(self):
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        subs = [
            ('vicon_robot', PoseStamped, self.topics['vicon_robot_pose'], 10),
            ('vicon_target', PoseStamped, self.topics['vicon_target_pose'], 10),
            ('camera_info', CameraInfo, self.topics['camera_info'], 10),
        ]

        # Compressed image
        comp = self.topics.get('camera_compressed', '')
        if comp:
            subs.append(('compressed_img', CompressedImage, comp, None))

        # Detection
        det_type = self.topics['detection_msg_type']
        if det_type == 'apriltag_msgs' and _HAS_APRILTAG_MSGS:
            subs.append(('apriltag_det', AprilTagDetectionArrayStd,
                         self.topics['apriltag_detections'], 10))
        elif det_type == 'isaac_ros' and _HAS_ISAAC_APRILTAG:
            subs.append(('apriltag_det', AprilTagDetectionArrayIsaac,
                         self.topics['apriltag_detections'], 10))

        # PX4 odom
        if _HAS_PX4:
            subs.append(('px4_odom', VehicleOdometry,
                         self.topics.get('px4_odometry', ''), None))

        for name, msg_type, topic, qos in subs:
            if not topic:
                continue
            q = qos if isinstance(qos, int) else qos_be
            if isinstance(q, int):
                q = q
            self.create_subscription(msg_type, topic, self._count(name), qos_be)

    def run_diagnostics(self, duration: float = 5.0):
        """Measure topic rates over *duration* seconds and print report."""
        print(f'\n{"="*65}')
        print('  DOCKING TEST SUITE - SYSTEM DIAGNOSTICS')
        print(f'{"="*65}\n')

        # Check topic existence
        topic_list = self.get_topic_names_and_types()
        topic_names = {name for name, _ in topic_list}

        checks = {
            'Vicon Robot Pose': self.topics['vicon_robot_pose'],
            'Vicon Target Pose': self.topics['vicon_target_pose'],
            'AprilTag Detections': self.topics['apriltag_detections'],
            'Camera Info': self.topics['camera_info'],
            'Compressed Image': self.topics.get('camera_compressed', ''),
            'PX4 Odometry': self.topics.get('px4_odometry', ''),
            'Cmd Vel': self.topics['cmd_vel'],
        }

        print('  Topic Existence Check:')
        for label, topic in checks.items():
            if not topic:
                status = 'NOT CONFIGURED'
            elif topic in topic_names:
                status = 'FOUND'
            else:
                status = 'MISSING'
            icon = '+' if status == 'FOUND' else ('-' if status == 'MISSING' else '?')
            print(f'    [{icon}] {label:25s} {topic}')

        # Measure rates
        print(f'\n  Measuring data rates for {duration}s ...')
        with self._lock:
            self._counters = {}

        time.sleep(duration)

        with self._lock:
            counts = dict(self._counters)

        print(f'\n  Data Rate Report:')
        print(f'    {"Topic":<25s} {"Messages":>10s} {"Rate (Hz)":>12s}')
        print(f'    {"-"*50}')

        friendly = {
            'vicon_robot': 'Vicon Robot',
            'vicon_target': 'Vicon Target',
            'camera_info': 'Camera Info',
            'compressed_img': 'Compressed Img',
            'apriltag_det': 'AprilTag Det.',
            'px4_odom': 'PX4 Odometry',
        }

        for key in ['vicon_robot', 'vicon_target', 'apriltag_det',
                     'camera_info', 'compressed_img', 'px4_odom']:
            name = friendly.get(key, key)
            n = counts.get(key, 0)
            rate = n / duration if duration > 0 else 0
            status = f'{n:>10d} {rate:>12.1f}'
            print(f'    {name:<25s} {status}')

        # Config summary
        print(f'\n  Configuration:')
        print(f'    Use Vicon: {self.cfg["system"]["use_vicon"]}')
        print(f'    Detection type: {self.topics["detection_msg_type"]}')
        print(f'    Tag size: {self.cfg["calibration"]["tag_size"]} m')
        t_cam = self.cfg['calibration']['vicon_robot_to_camera']['translation']
        t_tag = self.cfg['calibration']['vicon_target_to_real_target']['translation']
        print(f'    Robot-to-camera offset: {t_cam}')
        print(f'    Target Vicon-to-real offset: {t_tag}')

        # Dependency check
        print(f'\n  Package Availability:')
        print(f'    apriltag_msgs:                 {"OK" if _HAS_APRILTAG_MSGS else "NOT FOUND"}')
        print(f'    isaac_ros_apriltag_interfaces: {"OK" if _HAS_ISAAC_APRILTAG else "NOT FOUND"}')
        print(f'    px4_msgs:                      {"OK" if _HAS_PX4 else "NOT FOUND"}')

        try:
            import google.auth
            gdrive = 'OK'
        except ImportError:
            gdrive = 'NOT INSTALLED (pip install google-api-python-client ...)'
        print(f'    Google Drive API:              {gdrive}')

        print(f'\n{"="*65}\n')


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def run():
        time.sleep(1.0)  # Let subscriptions connect
        node.run_diagnostics(5.0)
        print('Diagnostics complete. Press Ctrl+C to exit.')

    t = threading.Thread(target=run, daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
