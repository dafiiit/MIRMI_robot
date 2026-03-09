"""Core data recording engine for the docking test suite.

Subscribes to Vicon, AprilTag, camera, and PX4 topics.  Provides
``start_recording`` / ``stop_recording`` methods that test scripts call
to capture time-windowed data into CSV + images.
"""

import csv
import os
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage

from .transforms import (
    pose_msg_to_matrix, build_offset_matrix,
    compute_ground_truth_relative_pose, estimate_tag_pose_pnp,
    compute_pose_error, matrix_to_pose,
)
from .config_loader import get_output_dir

# Try both detection message types
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

# PX4 odometry (optional)
try:
    from px4_msgs.msg import VehicleOdometry
    _HAS_PX4 = True
except ImportError:
    _HAS_PX4 = False


CSV_HEADER = [
    'timestamp_ns', 'elapsed_s', 'test_name', 'test_label',
    # Vicon robot (world frame)
    'vicon_robot_x', 'vicon_robot_y', 'vicon_robot_z',
    'vicon_robot_qx', 'vicon_robot_qy', 'vicon_robot_qz', 'vicon_robot_qw',
    # Vicon target (world frame)
    'vicon_target_x', 'vicon_target_y', 'vicon_target_z',
    'vicon_target_qx', 'vicon_target_qy', 'vicon_target_qz', 'vicon_target_qw',
    # Ground-truth relative pose (camera frame, with calibration offsets)
    'gt_rel_x', 'gt_rel_y', 'gt_rel_z',
    'gt_rel_qx', 'gt_rel_qy', 'gt_rel_qz', 'gt_rel_qw',
    # Camera estimate (solvePnP, camera frame)
    'cam_rel_x', 'cam_rel_y', 'cam_rel_z',
    'cam_rel_rvec_x', 'cam_rel_rvec_y', 'cam_rel_rvec_z',
    # AprilTag detection raw
    'tag_id',
    'tag_center_x', 'tag_center_y',
    'tag_c0_x', 'tag_c0_y',
    'tag_c1_x', 'tag_c1_y',
    'tag_c2_x', 'tag_c2_y',
    'tag_c3_x', 'tag_c3_y',
    # PX4 odometry fallback
    'px4_x', 'px4_y', 'px4_z',
    'px4_qw', 'px4_qx', 'px4_qy', 'px4_qz',
    # Error metrics
    'error_x', 'error_y', 'error_z', 'error_euclidean',
    # Image reference
    'image_filename',
]


def _nan():
    return float('nan')


class DataRecorder:
    """Attach to a ROS 2 Node and record synchronized sensor data.

    Usage (from a test node)::

        recorder = DataRecorder(self, config)
        recorder.start_recording('test_a_distance', 'distance_5m')
        time.sleep(10.0)
        csv_path = recorder.stop_recording()
    """

    def __init__(self, node: Node, config: dict):
        self.node = node
        self.cfg = config
        self.topics = config['topics']
        self.cal = config['calibration']
        self.rec_cfg = config['recording']
        self.sys_cfg = config['system']
        self.use_vicon = self.sys_cfg['use_vicon']

        # Calibration offset matrices
        self.T_robot_to_cam = build_offset_matrix(self.cal['vicon_robot_to_camera'])
        self.T_target_offset = build_offset_matrix(self.cal['vicon_target_to_real_target'])
        self.tag_size = self.cal['tag_size']

        # State
        self._recording = False
        self._test_name = ''
        self._test_label = ''
        self._start_time = None
        self._data_buffer = []
        self._lock = threading.Lock()

        # Camera intrinsics (populated on first CameraInfo message)
        self._camera_matrix = None
        self._dist_coeffs = None

        # Latest PX4 odom (for fallback)
        self._latest_px4_odom = None

        # Image saving
        self._image_dir = None
        self._last_image_save_time = 0.0
        self._image_min_interval = 1.0 / max(0.1, self.rec_cfg.get('image_save_rate', 2.0))
        self._latest_compressed = None

        # Output directory
        self._output_dir = get_output_dir(config)

        # ---------- Subscribers ----------

        # Camera info (always needed for solvePnP)
        self.node.create_subscription(
            CameraInfo,
            self.topics['camera_info'],
            self._camera_info_cb, 10,
        )

        # Compressed image (for saving annotated frames)
        compressed_topic = self.topics.get('camera_compressed', '')
        if compressed_topic:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=1,
            )
            self.node.create_subscription(
                CompressedImage, compressed_topic,
                self._compressed_cb, qos,
            )

        # PX4 odom fallback
        if self.sys_cfg.get('use_px4_odom_fallback', True) and _HAS_PX4:
            qos_px4 = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=1,
            )
            self.node.create_subscription(
                VehicleOdometry,
                self.topics.get('px4_odometry', '/fmu/out/vehicle_odometry'),
                self._px4_odom_cb, qos_px4,
            )

        # Detection message type
        det_type = self.topics['detection_msg_type']
        if det_type == 'apriltag_msgs':
            if not _HAS_APRILTAG_MSGS:
                raise ImportError('apriltag_msgs not installed but selected in config')
            self._DetectionMsg = AprilTagDetectionArrayStd
        else:
            if not _HAS_ISAAC_APRILTAG:
                raise ImportError('isaac_ros_apriltag_interfaces not installed but selected in config')
            self._DetectionMsg = AprilTagDetectionArrayIsaac

        # Set up synchronized or standalone subscriptions
        if self.use_vicon:
            self._setup_vicon_synced()
        else:
            self._setup_standalone()

    # ------------------------------------------------------------------
    # Subscription setups
    # ------------------------------------------------------------------

    def _setup_vicon_synced(self):
        """3-topic synchronization: vicon_robot + vicon_target + detections."""
        self._vicon_robot_sub = message_filters.Subscriber(
            self.node, PoseStamped, self.topics['vicon_robot_pose'])
        self._vicon_target_sub = message_filters.Subscriber(
            self.node, PoseStamped, self.topics['vicon_target_pose'])
        self._det_sub = message_filters.Subscriber(
            self.node, self._DetectionMsg, self.topics['apriltag_detections'])

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._vicon_robot_sub, self._vicon_target_sub, self._det_sub],
            queue_size=self.rec_cfg.get('sync_queue_size', 50),
            slop=self.rec_cfg.get('sync_slop', 0.5),
        )
        self._sync.registerCallback(self._synced_cb)

    def _setup_standalone(self):
        """Detection-only subscription (no Vicon)."""
        self.node.create_subscription(
            self._DetectionMsg,
            self.topics['apriltag_detections'],
            self._detection_only_cb, 10,
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d)
            self.node.get_logger().info('Camera intrinsics received')

    def _compressed_cb(self, msg: CompressedImage):
        self._latest_compressed = msg

    def _px4_odom_cb(self, msg):
        self._latest_px4_odom = msg

    def _synced_cb(self, robot_msg, target_msg, det_msg):
        """Synchronized callback: Vicon robot + Vicon target + AprilTag."""
        if not self._recording:
            return
        if not det_msg.detections:
            return
        if self._camera_matrix is None:
            return

        now_ns = self.node.get_clock().now().nanoseconds
        elapsed = (now_ns - self._start_time) / 1e9

        # Vicon world poses
        T_w_robot = pose_msg_to_matrix(robot_msg.pose)
        T_w_target = pose_msg_to_matrix(target_msg.pose)

        # Ground-truth relative pose in camera frame
        gt_pos, gt_q = compute_ground_truth_relative_pose(
            T_w_robot, T_w_target, self.T_robot_to_cam, self.T_target_offset)

        # Process first detection
        det = det_msg.detections[0]
        row = self._build_row(
            now_ns, elapsed, det,
            robot_msg.pose, target_msg.pose,
            gt_pos, gt_q,
        )

        with self._lock:
            self._data_buffer.append(row)

        self._maybe_save_image(now_ns)

    def _detection_only_cb(self, det_msg):
        """Standalone callback: AprilTag only (no Vicon)."""
        if not self._recording:
            return
        if not det_msg.detections:
            return
        if self._camera_matrix is None:
            return

        now_ns = self.node.get_clock().now().nanoseconds
        elapsed = (now_ns - self._start_time) / 1e9

        det = det_msg.detections[0]
        row = self._build_row(now_ns, elapsed, det)

        with self._lock:
            self._data_buffer.append(row)

        self._maybe_save_image(now_ns)

    # ------------------------------------------------------------------
    # Row construction
    # ------------------------------------------------------------------

    def _extract_detection(self, det):
        """Extract corners, center, and id from a detection message (either type)."""
        corners = [(c.x, c.y) for c in det.corners]
        tag_id = det.id

        if hasattr(det, 'center'):
            center = (det.center.x, det.center.y)
        else:
            cx = sum(c[0] for c in corners) / 4.0
            cy = sum(c[1] for c in corners) / 4.0
            center = (cx, cy)
        return tag_id, center, corners

    def _build_row(self, now_ns, elapsed, det,
                   robot_pose=None, target_pose=None,
                   gt_pos=None, gt_q=None):
        """Construct one CSV row dict."""
        tag_id, center, corners = self._extract_detection(det)

        # solvePnP
        tvec, rvec, _ = estimate_tag_pose_pnp(
            corners, self._camera_matrix, self._dist_coeffs, self.tag_size)

        # Vicon data
        nan = _nan()
        if robot_pose is not None:
            rp = robot_pose.position
            ro = robot_pose.orientation
            vr = [rp.x, rp.y, rp.z, ro.x, ro.y, ro.z, ro.w]
        else:
            vr = [nan] * 7

        if target_pose is not None:
            tp = target_pose.position
            to_ = target_pose.orientation
            vt = [tp.x, tp.y, tp.z, to_.x, to_.y, to_.z, to_.w]
        else:
            vt = [nan] * 7

        if gt_pos is not None:
            gt = list(gt_pos) + list(gt_q)
        else:
            gt = [nan] * 7

        if tvec is not None:
            cam = list(tvec) + list(rvec)
        else:
            cam = [nan] * 6

        # PX4 odom fallback
        if self._latest_px4_odom is not None:
            po = self._latest_px4_odom
            px4 = list(po.position) + list(po.q)
        else:
            px4 = [nan] * 7

        # Error
        if gt_pos is not None and tvec is not None:
            ex, ey, ez, ee = compute_pose_error(gt_pos, tvec)
            err = [ex, ey, ez, ee]
        else:
            err = [nan] * 4

        # Image filename placeholder (filled by _maybe_save_image)
        img_fn = ''

        row = {
            'timestamp_ns': now_ns,
            'elapsed_s': f'{elapsed:.6f}',
            'test_name': self._test_name,
            'test_label': self._test_label,
            'vicon_robot_x': vr[0], 'vicon_robot_y': vr[1], 'vicon_robot_z': vr[2],
            'vicon_robot_qx': vr[3], 'vicon_robot_qy': vr[4],
            'vicon_robot_qz': vr[5], 'vicon_robot_qw': vr[6],
            'vicon_target_x': vt[0], 'vicon_target_y': vt[1], 'vicon_target_z': vt[2],
            'vicon_target_qx': vt[3], 'vicon_target_qy': vt[4],
            'vicon_target_qz': vt[5], 'vicon_target_qw': vt[6],
            'gt_rel_x': gt[0], 'gt_rel_y': gt[1], 'gt_rel_z': gt[2],
            'gt_rel_qx': gt[3], 'gt_rel_qy': gt[4],
            'gt_rel_qz': gt[5], 'gt_rel_qw': gt[6],
            'cam_rel_x': cam[0], 'cam_rel_y': cam[1], 'cam_rel_z': cam[2],
            'cam_rel_rvec_x': cam[3], 'cam_rel_rvec_y': cam[4], 'cam_rel_rvec_z': cam[5],
            'tag_id': tag_id,
            'tag_center_x': center[0], 'tag_center_y': center[1],
            'tag_c0_x': corners[0][0], 'tag_c0_y': corners[0][1],
            'tag_c1_x': corners[1][0], 'tag_c1_y': corners[1][1],
            'tag_c2_x': corners[2][0], 'tag_c2_y': corners[2][1],
            'tag_c3_x': corners[3][0], 'tag_c3_y': corners[3][1],
            'px4_x': px4[0], 'px4_y': px4[1], 'px4_z': px4[2],
            'px4_qw': px4[3], 'px4_qx': px4[4], 'px4_qy': px4[5], 'px4_qz': px4[6],
            'error_x': err[0], 'error_y': err[1], 'error_z': err[2],
            'error_euclidean': err[3],
            'image_filename': img_fn,
        }
        return row

    # ------------------------------------------------------------------
    # Image saving
    # ------------------------------------------------------------------

    def _maybe_save_image(self, now_ns):
        """Save annotated compressed image at the configured rate."""
        if not self.rec_cfg.get('save_images', True):
            return
        if self._latest_compressed is None:
            return
        if self._image_dir is None:
            return

        now_s = now_ns / 1e9
        if now_s - self._last_image_save_time < self._image_min_interval:
            return
        self._last_image_save_time = now_s

        fname = f'frame_{now_ns}.jpg'
        fpath = os.path.join(self._image_dir, fname)
        with open(fpath, 'wb') as f:
            f.write(bytes(self._latest_compressed.data))

        # Tag the last row with this image filename
        with self._lock:
            if self._data_buffer:
                self._data_buffer[-1]['image_filename'] = fname

    # ------------------------------------------------------------------
    # Recording control API
    # ------------------------------------------------------------------

    def start_recording(self, test_name: str, test_label: str):
        """Begin a new recording session."""
        self._test_name = test_name
        self._test_label = test_label
        self._start_time = self.node.get_clock().now().nanoseconds

        # Create image subdirectory
        ts = time.strftime('%Y-%m-%d_%H-%M-%S')
        self._session_timestamp = ts
        if self.rec_cfg.get('save_images', True):
            self._image_dir = os.path.join(
                self._output_dir, f'{test_name}_{ts}_images')
            os.makedirs(self._image_dir, exist_ok=True)
        self._last_image_save_time = 0.0

        with self._lock:
            self._data_buffer = []
        self._recording = True

        self.node.get_logger().info(
            f'Recording STARTED: {test_name} / {test_label}')

    def update_label(self, test_label: str):
        """Update the label mid-recording (e.g. for each distance step)."""
        self._test_label = test_label

    def stop_recording(self) -> str:
        """Stop recording and write the CSV file.

        Returns the path to the saved CSV.
        """
        self._recording = False

        with self._lock:
            data = list(self._data_buffer)
            self._data_buffer = []

        ts = getattr(self, '_session_timestamp', time.strftime('%Y-%m-%d_%H-%M-%S'))
        csv_name = f'{self._test_name}_{ts}.csv'
        csv_path = os.path.join(self._output_dir, csv_name)

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
            writer.writeheader()
            writer.writerows(data)

        n = len(data)
        self.node.get_logger().info(
            f'Recording STOPPED: {n} samples saved to {csv_path}')
        return csv_path

    @property
    def sample_count(self) -> int:
        with self._lock:
            return len(self._data_buffer)

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def latest_cam_distance(self) -> float:
        """Return the latest camera-estimated distance to the tag, or NaN."""
        with self._lock:
            if not self._data_buffer:
                return _nan()
            last = self._data_buffer[-1]
            x = last.get('cam_rel_x', _nan())
            y = last.get('cam_rel_y', _nan())
            z = last.get('cam_rel_z', _nan())
            try:
                return float(np.sqrt(float(x)**2 + float(y)**2 + float(z)**2))
            except (ValueError, TypeError):
                return _nan()
