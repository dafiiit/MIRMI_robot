"""Multi-sensor data recorder for the sweep test suite.

Extends the logic of the original DataRecorder with:
  - LiDAR detection subscriber (/station_confidence + /station_marker)
  - Sensor-gated subscriptions (camera / lidar / both)
  - rosbag2 subprocess recording (raw topics)
  - Per-step CSV files with timestamped filenames
  - rclone upload helper

CSV columns are documented in CSV_HEADER below.
"""

import csv
import json
import os
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

from .transforms import estimate_tag_pose_pnp
from .config_loader import get_output_dir

# ── Optional imports ────────────────────────────────────────────────────────

try:
    from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
    _HAS_ISAAC_APRILTAG = True
except ImportError:
    _HAS_ISAAC_APRILTAG = False

try:
    from apriltag_msgs.msg import AprilTagDetectionArray as AprilTagDetectionArrayStd
    _HAS_APRILTAG_MSGS = True
except ImportError:
    _HAS_APRILTAG_MSGS = False

try:
    from px4_msgs.msg import VehicleOdometry
    _HAS_PX4 = True
except ImportError:
    _HAS_PX4 = False

# ── CSV schema ───────────────────────────────────────────────────────────────

CSV_HEADER = [
    # Identity
    'timestamp_ns', 'elapsed_s',
    'step_label', 'mode', 'scenario', 'sensors',
    # AprilTag (camera)
    'tag_id',
    'tag_center_x', 'tag_center_y',
    'tag_c0_x', 'tag_c0_y',
    'tag_c1_x', 'tag_c1_y',
    'tag_c2_x', 'tag_c2_y',
    'tag_c3_x', 'tag_c3_y',
    # Camera pose estimate (solvePnP)
    'cam_rel_x', 'cam_rel_y', 'cam_rel_z',
    'cam_rel_rvec_x', 'cam_rel_rvec_y', 'cam_rel_rvec_z',
    'cam_distance_m',
    # LiDAR detections
    'lidar_detected',
    'lidar_confidence',
    'lidar_center_x', 'lidar_center_y',
    'lidar_distance_m',
    # PX4 odometry
    'px4_x', 'px4_y', 'px4_z',
    'px4_qw', 'px4_qx', 'px4_qy', 'px4_qz',
    # Image reference
    'image_filename',
]


def _nan():
    return float('nan')


# ── Main class ───────────────────────────────────────────────────────────────

class SweepRecorder:
    """Record synchronized multi-sensor data for one sweep step.

    Usage::

        rec = SweepRecorder(node, config, sensors='both')
        rec.start('out_dist_08p0m', 'distance', 'outdoor', output_dir)
        time.sleep(10.0)
        csv_path, bag_dir = rec.stop()
    """

    # Topics always added to every rosbag recording
    _ALWAYS_BAG_TOPICS = [
        '/sweep_test/status',
        '/sweep_test/capturing',
        '/sweep_test/target_placement',
    ]

    def __init__(self, node: Node, config: dict, sensors: str = 'both'):
        """
        Parameters
        ----------
        node : rclpy.node.Node
        config : dict
            Parsed test_config.yaml
        sensors : str
            'camera', 'lidar', or 'both'
        """
        self.node = node
        self.cfg = config
        self.sensors = sensors  # updated by caller before start()
        self._sc = config.get('sweep_test', {}).get('sensors', {})
        self._rec_cfg = config.get('recording', {})

        # State
        self._recording = False
        self._abort = False
        self._step_label = ''
        self._mode = ''
        self._scenario = ''
        self._start_ns = None
        self._lock = threading.Lock()
        self._buffer = []

        # Camera intrinsics
        self._camera_matrix = None
        self._dist_coeffs = None

        # Latest sensor snapshots
        self._latest_compressed = None
        self._last_image_ns = 0.0
        self._image_min_interval = 1.0 / max(0.1, self._rec_cfg.get('image_save_rate', 2.0))
        self._image_dir = None

        self._latest_px4_odom = None

        # LiDAR detection state (from /station_confidence + /station_marker)
        self._lidar_confidence = _nan()
        self._lidar_center_x = _nan()
        self._lidar_center_y = _nan()
        self._lidar_detected = False

        # rosbag subprocess
        self._bag_proc = None

        # ── Subscribers ──────────────────────────────────────────────────

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Camera info — always needed for solvePnP
        cam_info_topic = self._sc.get('camera_info_topic',
                                       '/camera/camera/color/camera_info')
        self.node.create_subscription(
            CameraInfo, cam_info_topic, self._camera_info_cb, 10)

        # Compressed image overlay (for JPEG saving)
        overlay_topic = self._sc.get('camera_overlay_topic',
                                      '/apriltag/overlay/compressed')
        self.node.create_subscription(
            CompressedImage, overlay_topic, self._compressed_cb, qos_be)

        # AprilTag detections
        det_type = config['topics'].get('detection_msg_type', 'isaac_ros')
        apriltag_topic = self._sc.get('apriltag_topic',
                                       config['topics'].get('apriltag_detections',
                                                            '/tag_detections'))
        if det_type == 'isaac_ros' and _HAS_ISAAC_APRILTAG:
            self.node.create_subscription(
                AprilTagDetectionArray, apriltag_topic,
                self._apriltag_cb, 10)
        elif _HAS_APRILTAG_MSGS:
            self.node.create_subscription(
                AprilTagDetectionArrayStd, apriltag_topic,
                self._apriltag_cb, 10)
        else:
            self.node.get_logger().warn(
                'No AprilTag message package found; camera data will be NaN.')

        # LiDAR confidence + marker
        self.node.create_subscription(
            Float32, '/station_confidence',
            self._lidar_conf_cb, qos_be)
        self.node.create_subscription(
            Marker, '/station_marker',
            self._lidar_marker_cb, qos_be)

        # PX4 odometry
        if _HAS_PX4:
            px4_topic = config['topics'].get('px4_odometry',
                                              '/fmu/out/vehicle_odometry')
            self.node.create_subscription(
                VehicleOdometry, px4_topic,
                self._px4_odom_cb, qos_be)

    # ── Subscriber callbacks ─────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d)
            self.node.get_logger().info('[SweepRecorder] Camera intrinsics received')

    def _compressed_cb(self, msg: CompressedImage):
        self._latest_compressed = msg

    def _px4_odom_cb(self, msg):
        self._latest_px4_odom = msg

    def _lidar_conf_cb(self, msg: Float32):
        self._lidar_confidence = float(msg.data)
        self._lidar_detected = (self._lidar_confidence > 0.0)

    def _lidar_marker_cb(self, msg: Marker):
        self._lidar_center_x = float(msg.pose.position.x)
        self._lidar_center_y = float(msg.pose.position.y)
        self._lidar_detected = True

    def _apriltag_cb(self, det_msg):
        """Called on every AprilTag detection array; records a row if active."""
        if not self._recording or self._abort:
            return
        if not det_msg.detections:
            return

        now_ns = self.node.get_clock().now().nanoseconds
        elapsed = (now_ns - self._start_ns) / 1e9

        det = det_msg.detections[0]
        row = self._build_row(now_ns, elapsed, det)

        with self._lock:
            self._buffer.append(row)

        self._maybe_save_image(now_ns)

    # ── Row construction ─────────────────────────────────────────────────────

    def _extract_detection(self, det):
        corners = [(c.x, c.y) for c in det.corners]
        tag_id = det.id
        if hasattr(det, 'center'):
            center = (det.center.x, det.center.y)
        else:
            center = (sum(c[0] for c in corners) / 4.0,
                      sum(c[1] for c in corners) / 4.0)
        return tag_id, center, corners

    def _build_row(self, now_ns, elapsed, det):
        nan = _nan()
        tag_id, center, corners = self._extract_detection(det)

        # solvePnP
        tvec, rvec, ok = (None, None, False)
        cal = self.cfg.get('calibration', {})
        tag_size = cal.get('tag_size', 0.162)
        if self._camera_matrix is not None:
            tvec, rvec, ok = estimate_tag_pose_pnp(
                corners, self._camera_matrix, self._dist_coeffs, tag_size)

        if tvec is not None:
            cam = list(tvec) + list(rvec)
            cam_dist = float(np.linalg.norm(tvec))
        else:
            cam = [nan] * 6
            cam_dist = nan

        # PX4
        if self._latest_px4_odom is not None:
            po = self._latest_px4_odom
            px4 = list(po.position) + list(po.q)
        else:
            px4 = [nan] * 7

        # LiDAR
        if self._lidar_detected:
            lidar_dist = float(np.hypot(self._lidar_center_x, self._lidar_center_y))
        else:
            lidar_dist = nan

        # Pad corners to 4
        while len(corners) < 4:
            corners.append((nan, nan))

        row = {
            'timestamp_ns':   now_ns,
            'elapsed_s':      f'{elapsed:.6f}',
            'step_label':     self._step_label,
            'mode':           self._mode,
            'scenario':       self._scenario,
            'sensors':        self.sensors,
            # AprilTag
            'tag_id':         tag_id,
            'tag_center_x':   center[0], 'tag_center_y': center[1],
            'tag_c0_x': corners[0][0], 'tag_c0_y': corners[0][1],
            'tag_c1_x': corners[1][0], 'tag_c1_y': corners[1][1],
            'tag_c2_x': corners[2][0], 'tag_c2_y': corners[2][1],
            'tag_c3_x': corners[3][0], 'tag_c3_y': corners[3][1],
            # Camera pose
            'cam_rel_x': cam[0], 'cam_rel_y': cam[1], 'cam_rel_z': cam[2],
            'cam_rel_rvec_x': cam[3], 'cam_rel_rvec_y': cam[4], 'cam_rel_rvec_z': cam[5],
            'cam_distance_m': cam_dist,
            # LiDAR
            'lidar_detected':    int(self._lidar_detected),
            'lidar_confidence':  self._lidar_confidence,
            'lidar_center_x':    self._lidar_center_x,
            'lidar_center_y':    self._lidar_center_y,
            'lidar_distance_m':  lidar_dist,
            # PX4
            'px4_x': px4[0], 'px4_y': px4[1], 'px4_z': px4[2],
            'px4_qw': px4[3], 'px4_qx': px4[4], 'px4_qy': px4[5], 'px4_qz': px4[6],
            # image placeholder
            'image_filename': '',
        }
        return row

    # ── Image saving ─────────────────────────────────────────────────────────

    def _maybe_save_image(self, now_ns):
        if self.sensors == 'lidar':
            return
        if self._latest_compressed is None or self._image_dir is None:
            return
        now_s = now_ns / 1e9
        if now_s - self._last_image_ns < self._image_min_interval:
            return
        self._last_image_ns = now_s

        fname = f'frame_{now_ns}.jpg'
        fpath = os.path.join(self._image_dir, fname)
        try:
            with open(fpath, 'wb') as f:
                f.write(bytes(self._latest_compressed.data))
        except Exception as e:
            self.node.get_logger().warn(f'[SweepRecorder] Image save failed: {e}')
            return

        with self._lock:
            if self._buffer:
                self._buffer[-1]['image_filename'] = fname

    # ── rosbag subprocess ────────────────────────────────────────────────────

    def _bag_topics_for_sensors(self):
        """Return list of topic strings to record based on selected sensors."""
        sc = self._sc
        topics = list(self._ALWAYS_BAG_TOPICS)
        if self.sensors in ('camera', 'both'):
            topics += [
                sc.get('camera_raw_topic',     '/camera/camera/color/image_raw'),
                sc.get('camera_info_topic',    '/camera/camera/color/camera_info'),
                sc.get('camera_overlay_topic', '/apriltag/overlay/compressed'),
                sc.get('apriltag_topic',       '/tag_detections'),
            ]
        if self.sensors in ('lidar', 'both'):
            topics += [
                sc.get('lidar_raw_topic',       '/livox/lidar'),
                sc.get('lidar_detection_topic', '/station_confidence'),
                '/station_marker',
            ]
        # Deduplicate
        seen = set()
        return [t for t in topics if not (t in seen or seen.add(t))]

    def _start_bag(self, bag_dir: str):
        """Spawn ros2 bag record as a subprocess."""
        topics = self._bag_topics_for_sensors()
        cmd = ['ros2', 'bag', 'record', '--output', bag_dir] + topics
        self.node.get_logger().info(
            f'[SweepRecorder] Starting bag: {bag_dir}\n  topics: {topics}')
        try:
            self._bag_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,  # own process group so we can SIGINT cleanly
            )
        except Exception as e:
            self.node.get_logger().error(f'[SweepRecorder] Failed to start bag: {e}')
            self._bag_proc = None

    def _stop_bag(self):
        """Send SIGINT to the bag process group and wait."""
        if self._bag_proc is None:
            return
        try:
            os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)
            self._bag_proc.wait(timeout=10)
        except Exception as e:
            self.node.get_logger().warn(f'[SweepRecorder] Bag stop issue: {e}')
            try:
                self._bag_proc.kill()
            except Exception:
                pass
        finally:
            self._bag_proc = None

    # ── Public API ───────────────────────────────────────────────────────────

    def start(self, step_label: str, scenario: str, mode: str, output_dir: str):
        """Begin recording for one sweep step.

        Parameters
        ----------
        step_label : str
            e.g. 'dist_08p0m'
        scenario : str
            'distance' or 'angular'
        mode : str
            'indoor' or 'outdoor'
        output_dir : str
            Expanded absolute path to the output directory.
        """
        ts = time.strftime('%Y%m%d_%H%M%S')
        mode_short = 'in' if mode == 'indoor' else 'out'
        scen_short = 'dist' if scenario == 'distance' else 'ang'
        base_name = f'{ts}_{mode_short}_{scen_short}_{step_label}'

        self._step_label = step_label
        self._scenario = scenario
        self._mode = mode
        self._start_ns = self.node.get_clock().now().nanoseconds
        self._abort = False

        # Reset LiDAR snapshot
        self._lidar_confidence = _nan()
        self._lidar_center_x = _nan()
        self._lidar_center_y = _nan()
        self._lidar_detected = False

        # Image directory
        if self.sensors in ('camera', 'both'):
            self._image_dir = os.path.join(output_dir, f'{base_name}_images')
            os.makedirs(self._image_dir, exist_ok=True)
        else:
            self._image_dir = None
        self._last_image_ns = 0.0

        with self._lock:
            self._buffer = []

        # rosbag
        bag_dir = os.path.join(output_dir, f'{base_name}_bag')
        self._start_bag(bag_dir)
        self._current_bag_dir = bag_dir
        self._current_base_name = base_name
        self._current_output_dir = output_dir

        self._recording = True
        self.node.get_logger().info(
            f'[SweepRecorder] Recording STARTED: {step_label}')

    def stop(self):
        """Stop recording and write CSV.

        Returns
        -------
        (csv_path, bag_dir) : (str, str)
        """
        self._recording = False
        self._stop_bag()

        with self._lock:
            data = list(self._buffer)
            self._buffer = []

        base_name = self._current_base_name
        out_dir = self._current_output_dir
        csv_path = os.path.join(out_dir, f'{base_name}.csv')

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
            writer.writeheader()
            writer.writerows(data)

        n = len(data)
        self.node.get_logger().info(
            f'[SweepRecorder] Recording STOPPED: {n} samples → {csv_path}')
        return csv_path, self._current_bag_dir

    def abort(self):
        """Discard current recording without saving."""
        self._abort = True
        self._recording = False
        self._stop_bag()
        with self._lock:
            self._buffer = []
        self.node.get_logger().warn('[SweepRecorder] Recording ABORTED')

    @property
    def sample_count(self) -> int:
        with self._lock:
            return len(self._buffer)

    @property
    def is_recording(self) -> bool:
        return self._recording


# ── rclone upload helper ─────────────────────────────────────────────────────

def rclone_upload(local_path: str, remote: str, remote_folder: str,
                   logger=None) -> bool:
    """Upload *local_path* (file or dir) to rclone remote.

    Returns True on success.
    """
    dest = f'{remote}:{remote_folder}/'
    cmd = ['rclone', 'copy', local_path, dest, '--progress']
    log = logger.info if logger else print
    warn = logger.warn if logger else print
    log(f'[rclone] Uploading {local_path} → {dest}')
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
        if result.returncode == 0:
            log(f'[rclone] Upload OK: {local_path}')
            return True
        else:
            warn(f'[rclone] Upload FAILED (rc={result.returncode}): {result.stderr}')
            return False
    except FileNotFoundError:
        warn('[rclone] rclone not found — is it installed?')
        return False
    except subprocess.TimeoutExpired:
        warn('[rclone] Upload timed out after 300 s')
        return False
    except Exception as e:
        warn(f'[rclone] Unexpected error: {e}')
        return False
