"""Sensor Sweep Test Node — topic-driven, no robot driving.

State machine
─────────────
  IDLE
    │  /sweep_test/cmd/configure (JSON)
    ▼
  STARTING_SENSORS  (auto-starts APRILTAG / LIDAR if needed, waits grace period)
    │  (grace period elapsed)
    ▼
  CAPTURING  ── (abort) ──► CONFIGURED
    │  (capture_duration elapsed)
    ▼
  UPLOADING
    │  (rclone done)
    ▼
  DONE   or   CONFIGURED  (ready for next capture in same sweep folder)
    │  /sweep_test/cmd/configure to start a new capture
    ▼
  STARTING_SENSORS / CAPTURING ...

Published topics
────────────────
  /sweep_test/status           String (JSON)  2 Hz
  /sweep_test/target_placement String          on change
  /sweep_test/capturing        Bool            continuously
  /sweep_test/step_complete    String (JSON)   once per step

Subscribed topics (trigger / command)
──────────────────────────────────────
  /sweep_test/cmd/configure    String (JSON)   → auto-starts sensors + capture
  /sweep_test/cmd/abort        Empty

Usage (from shell)
──────────────────
  ros2 launch docking_test_suite sensor_sweep.launch.py

  # Configure and immediately start capture (outdoor, both sensors, 10m, 0°)
  ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \\
    'data: "{\"mode\":\"outdoor\",\"scenario\":\"custom\",\"sensors\":\"both\",\"distance\":10.0,\"angle\":0.0}"'

  # Watch state + placement instructions
  ros2 topic echo /sweep_test/status
  ros2 topic echo /sweep_test/target_placement

  # Abort an ongoing capture
  ros2 topic pub --once /sweep_test/cmd/abort std_msgs/Empty '{}'
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String, Bool, Empty

from .config_loader import load_config, get_output_dir
from .sweep_recorder import SweepRecorder, rclone_upload
from .sweep_scenarios import distance_sweep_steps, angular_sweep_steps
from .gdrive_uploader import maybe_upload   # OAuth2 fallback


# ── State constants ──────────────────────────────────────────────────────────

STATE_IDLE             = 'IDLE'
STATE_STARTING_SENSORS = 'STARTING_SENSORS'
STATE_CONFIGURED       = 'CONFIGURED'
STATE_CAPTURING        = 'CAPTURING'
STATE_UPLOADING        = 'UPLOADING'
STATE_DONE             = 'DONE'


def _best_effort_qos(depth: int = 1) -> QoSProfile:
    """BEST_EFFORT QoS — compatible with Foxglove and most ROS 2 publishers."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class SensorSweepNode(Node):
    """Topic-driven sensor sweep test node.

    Sending a configure message is the only required step:
      1. Node auto-starts the required sensor detectors (if not already running).
      2. Waits a configurable grace period (sensor_startup_grace_s).
      3. Starts data capture automatically.
      4. Stops after capture_duration seconds and saves CSV + bag.
    """

    def __init__(self):
        super().__init__('sensor_sweep_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('config_path', '')
        cp = self.get_parameter('config_path').get_parameter_value().string_value
        self.cfg = load_config(cp if cp else None)

        sweep_cfg = self.cfg.get('sweep_test', {})
        self._capture_duration = float(sweep_cfg.get('capture_duration', 10.0))
        self._output_dir = os.path.expanduser(
            sweep_cfg.get('output_dir', '~/sweep_test_data'))
        os.makedirs(self._output_dir, exist_ok=True)

        self._auto_start_sensors = bool(
            sweep_cfg.get('auto_start_sensors', True))
        self._sensor_startup_grace_s = float(
            sweep_cfg.get('sensor_startup_grace_s', 5.0))

        # ── State ────────────────────────────────────────────────────────
        self._state = STATE_IDLE
        self._mode = ''
        self._scenario = ''
        self._sensors = 'both'
        self._distance = 0.0
        self._angle = 0.0
        self._steps = []
        self._step_index = 0
        self._capture_start_ns = None
        self._last_csv = ''
        self._last_bag = ''
        self._current_sweep_dir = self._output_dir
        self._gdrive_ok = None   # None = not attempted, True/False = result
        self._state_lock = threading.Lock()

        # Track which sensors we started so we don't double-start
        self._started_apriltag = False
        self._started_lidar = False

        # ── Recorder (created once, reused per step) ─────────────────────
        self._recorder = SweepRecorder(self, self.cfg, sensors='both')

        # ── Publishers ───────────────────────────────────────────────────
        self._pub_status = self.create_publisher(String, '/sweep_test/status', 10)
        self._pub_placement = self.create_publisher(
            String, '/sweep_test/target_placement', 10)
        self._pub_capturing = self.create_publisher(
            Bool, '/sweep_test/capturing', 10)
        self._pub_step_done = self.create_publisher(
            String, '/sweep_test/step_complete', 10)
        self._pub_upload_status = self.create_publisher(
            String, '/sweep_test/upload_status', 10)

        # Publishers for auto-starting sensor detectors via ProcessManager
        self._pub_start_apriltag = self.create_publisher(
            Empty, '/start_station_detection_APRILTAG', 10)
        self._pub_start_lidar = self.create_publisher(
            Empty, '/start_station_detection_LIDAR', 10)

        # ── Subscribers — ALL use BEST_EFFORT to match Foxglove ──────────
        be_qos = _best_effort_qos(depth=10)

        self.create_subscription(
            String, '/sweep_test/cmd/configure',
            self._cmd_configure_cb, be_qos)
        self.create_subscription(
            Empty, '/sweep_test/cmd/abort',
            self._cmd_abort_cb, be_qos)
        self.create_subscription(
            Empty, '/sweep_test/cmd/start_data_upload',
            self._cmd_start_data_upload_cb, be_qos)

        # ── Status heartbeat timer (2 Hz) ────────────────────────────────
        self._status_timer = self.create_timer(0.5, self._status_timer_cb)

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════╗\n'
            '║         SENSOR SWEEP TEST NODE  —  READY             ║\n'
            '╠══════════════════════════════════════════════════════╣\n'
            '║  Configure:    /sweep_test/cmd/configure    (JSON)   ║\n'
            '║    → auto-starts sensors + capture automatically     ║\n'
            '║  Abort:        /sweep_test/cmd/abort        (Empty)  ║\n'
            '║  Upload:       /sweep_test/cmd/start_data_upload     ║\n'
            '║  Status:       /sweep_test/status           (echo)   ║\n'
            '║  Upload stat:  /sweep_test/upload_status    (echo)   ║\n'
            '║  Placement:    /sweep_test/target_placement (echo)   ║\n'
            '╚══════════════════════════════════════════════════════╝\n'
            '\nSend a configure message to begin (capture starts automatically):\n'
            '  ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \\\n'
            '    \'data: "{\\\"mode\\\":\\\"outdoor\\\",\\\"scenario\\\":\\\"custom\\\",\\\"sensors\\\":\\\"both\\\",\\\"distance\\\":10.0,\\\"angle\\\":0.0}"\''
        )

    # ── Command callbacks ────────────────────────────────────────────────────

    def _cmd_configure_cb(self, msg: String):
        """Parse JSON configure message → auto-start sensors → auto-capture."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'[configure] Invalid JSON: {e}\n'
                f'  Expected: {{"mode":"outdoor","scenario":"custom","sensors":"both","distance":10.0,"angle":0.0}}')
            return

        mode     = data.get('mode', 'outdoor')
        scenario = data.get('scenario', 'custom')
        sensors  = data.get('sensors', 'both')
        distance = float(data.get('distance', 0.0))
        angle    = float(data.get('angle', 0.0))

        # Validate
        if mode not in ('indoor', 'outdoor'):
            self.get_logger().error(
                f'[configure] Invalid mode "{mode}". Use "indoor" or "outdoor".')
            return
        if scenario not in ('distance', 'angular', 'custom'):
            self.get_logger().error(
                f'[configure] Invalid scenario "{scenario}". Use "distance", "angular", or "custom".')
            return
        if sensors not in ('camera', 'lidar', 'both'):
            self.get_logger().error(
                f'[configure] Invalid sensors "{sensors}". Use "camera", "lidar", or "both".')
            return

        with self._state_lock:
            if self._state == STATE_CAPTURING:
                self.get_logger().warn(
                    '[configure] Capture in progress — abort first with /sweep_test/cmd/abort')
                return
            if self._state == STATE_STARTING_SENSORS:
                self.get_logger().warn(
                    '[configure] Already starting sensors, please wait.')
                return

            self._mode     = mode
            self._scenario = scenario
            self._sensors  = sensors
            self._distance = distance
            self._angle    = angle
            self._recorder.sensors = sensors
            self._gdrive_ok = None

            # Build single step
            sign  = 'p' if angle >= 0 else 'n'
            label = f"dist_{distance:04.1f}m_ang_{sign}{abs(int(angle)):03d}deg".replace('.', 'p')
            instruction = (
                f"[MEASUREMENT] "
                f"Place the target at {distance:.1f} m and {angle}°."
            )
            self._steps = [{
                'label':       label,
                'value':       distance,
                'instruction': instruction,
                'scenario':    scenario,
                'distance':    distance,
                'angle':       angle,
            }]
            self._step_index = 0

            # Generate folder structure for this sweep (new folder per session)
            if (not getattr(self, '_current_sweep_dir', None)
                    or self._current_sweep_dir == self._output_dir
                    or getattr(self, '_last_mode', None) != mode
                    or getattr(self, '_last_scenario', None) != scenario
                    or getattr(self, '_last_sensors', None) != sensors):
                ts = time.strftime('%Y%m%d_%H%M%S')
                mode_short = 'in' if mode == 'indoor' else 'out'
                scen_short = ('dist' if scenario == 'distance'
                              else ('ang' if scenario == 'angular' else 'cust'))
                sweep_folder_name = f'{ts}_{mode_short}_{scen_short}_{sensors}'
                main_folder = (
                    'camera_series'   if sensors == 'camera'  else
                    'lidar_series'    if sensors == 'lidar'   else
                    'combined_series'
                )
                self._current_sweep_dir = os.path.join(
                    self._output_dir, main_folder, sweep_folder_name)
                os.makedirs(self._current_sweep_dir, exist_ok=True)
                self._last_mode     = mode
                self._last_scenario = scenario
                self._last_sensors  = sensors

            self._state   = STATE_CONFIGURED
            self._last_csv = ''
            self._last_bag = ''

        self.get_logger().info(
            f'[configure] ✓ mode={mode}  scenario={scenario}  '
            f'sensors={sensors}  distance={distance}m  angle={angle}deg  '
            f'step={label}')

        self._publish_placement()

        # ── Auto-start sensors + capture in background ───────────────────
        t = threading.Thread(target=self._auto_sensor_and_capture_worker,
                             args=(sensors,), daemon=True)
        t.start()

    def _cmd_abort_cb(self, _msg: Empty):
        """Abort an ongoing capture without saving."""
        with self._state_lock:
            if self._state not in (STATE_CAPTURING, STATE_STARTING_SENSORS):
                self.get_logger().info(
                    f'[abort] Nothing to abort (state={self._state})')
                return
            self._state = STATE_CONFIGURED

        self._recorder.abort()
        self._publish_capturing(False)
        self.get_logger().warn('[abort] ✗ Capture ABORTED — data discarded.')
        self._publish_placement()

    # ── Auto sensor-start + capture worker ──────────────────────────────────

    def _auto_sensor_and_capture_worker(self, sensors: str):
        """Background thread: auto-start sensors if needed, then capture."""
        grace_needed = False

        if self._auto_start_sensors:
            needs_camera = sensors in ('camera', 'both')
            needs_lidar  = sensors in ('lidar', 'both')

            if needs_camera and not self._is_apriltag_active():
                self.get_logger().info(
                    '[auto-start] AprilTag detector not active — requesting start...')
                self._pub_start_apriltag.publish(Empty())
                self._started_apriltag = True
                grace_needed = True
            else:
                self.get_logger().info(
                    '[auto-start] AprilTag detector already active — skipping start.')

            if needs_lidar and not self._is_lidar_active():
                self.get_logger().info(
                    '[auto-start] LiDAR detector not active — requesting start...')
                self._pub_start_lidar.publish(Empty())
                self._started_lidar = True
                grace_needed = True
            else:
                if needs_lidar:
                    self.get_logger().info(
                        '[auto-start] LiDAR detector already active — skipping start.')

        if grace_needed:
            with self._state_lock:
                if self._state != STATE_CONFIGURED:
                    return  # aborted
                self._state = STATE_STARTING_SENSORS

            self.get_logger().info(
                f'[auto-start] Waiting {self._sensor_startup_grace_s:.0f} s '
                f'for sensors to spin up...')

            for _ in range(int(self._sensor_startup_grace_s * 10)):
                time.sleep(0.1)
                with self._state_lock:
                    if self._state != STATE_STARTING_SENSORS:
                        return  # aborted during grace period

            with self._state_lock:
                if self._state != STATE_STARTING_SENSORS:
                    return
                self._state = STATE_CONFIGURED

        # Now trigger capture
        self._start_capture()

    def _is_apriltag_active(self) -> bool:
        """Return True if the AprilTag detector appears to be publishing."""
        try:
            publishers = self.get_publishers_info_by_topic('/tag_detections')
            return len(publishers) > 0
        except Exception:
            return False

    def _is_lidar_active(self) -> bool:
        """Return True if the LiDAR detector appears to be publishing."""
        try:
            pub_conf   = self.get_publishers_info_by_topic('/station_confidence')
            pub_marker = self.get_publishers_info_by_topic('/station_marker')
            return len(pub_conf) > 0 or len(pub_marker) > 0
        except Exception:
            return False

    # ── Capture ──────────────────────────────────────────────────────────────

    def _start_capture(self):
        """Transition to CAPTURING and begin recording."""
        with self._state_lock:
            if self._state != STATE_CONFIGURED:
                self.get_logger().warn(
                    f'[capture] Cannot start capture in state {self._state}.')
                return
            if not self._steps:
                self.get_logger().error('[capture] No steps defined — configure first.')
                return

            self._state = STATE_CAPTURING
            self._capture_start_ns = self.get_clock().now().nanoseconds

        step = self._steps[self._step_index]
        self.get_logger().info(
            f'\n{"═"*60}\n'
            f'  CAPTURING  step {self._step_index + 1}/{len(self._steps)}\n'
            f'  label:    {step["label"]}\n'
            f'  mode:     {self._mode}  |  sensors: {self._sensors}\n'
            f'  duration: {self._capture_duration:.0f} s\n'
            f'{"═"*60}'
        )

        self._publish_capturing(True)

        self._recorder.start(
            step_label=step['label'],
            scenario=self._scenario,
            mode=self._mode,
            output_dir=self._current_sweep_dir,
        )

        t = threading.Thread(target=self._capture_worker, daemon=True)
        t.start()

    def _capture_worker(self):
        """Background thread: wait capture_duration, then finalize."""
        interval = 1.0
        elapsed  = 0.0
        while elapsed < self._capture_duration:
            time.sleep(min(interval, self._capture_duration - elapsed))
            elapsed += interval

            with self._state_lock:
                if self._state != STATE_CAPTURING:
                    return   # aborted

            if int(elapsed) % 2 == 0:
                self.get_logger().info(
                    f'  ⏱  {elapsed:.0f}/{self._capture_duration:.0f} s  '
                    f'| {self._recorder.sample_count} samples')

        self._finalize_capture()

    def _finalize_capture(self):
        """Stop recorder, write files, upload, advance step."""
        with self._state_lock:
            if self._state != STATE_CAPTURING:
                return
            self._state = STATE_UPLOADING

        self._publish_capturing(False)

        csv_path, bag_dir = self._recorder.stop()
        step = self._steps[self._step_index]

        # Count actual rows written
        try:
            import csv as _csv
            with open(csv_path, 'r') as f:
                n = sum(1 for _ in f) - 1   # minus header
        except Exception:
            n = -1

        self._last_csv = csv_path
        self._last_bag = bag_dir

        self.get_logger().info(
            f'\n{"═"*60}\n'
            f'  CAPTURE COMPLETE  step {self._step_index + 1}/{len(self._steps)}\n'
            f'  samples:  {n}\n'
            f'  CSV:      {csv_path}\n'
            f'  bag dir:  {bag_dir}\n'
            f'{"═"*60}'
        )

        step_done_msg = json.dumps({
            'step_label':  step['label'],
            'samples':     n,
            'images':      self._recorder.image_count,
            'pointclouds': self._recorder.pointcloud_count,
            'csv_file':    csv_path,
            'bag_dir':     bag_dir,
            'duration_s':  self._capture_duration,
        })
        self._pub_step_done.publish(String(data=step_done_msg))

        with self._state_lock:
            self._step_index += 1
            if self._step_index >= len(self._steps):
                self._state = STATE_DONE
                self.get_logger().info(
                    '\n╔══════════════════════════════════════╗\n'
                    '║   ALL STEPS COMPLETE — sweep DONE!   ║\n'
                    '║   Send /configure for next capture.   ║\n'
                    '╚══════════════════════════════════════╝')
            else:
                self._state = STATE_CONFIGURED

        self._publish_placement()

    # ── Upload worker ────────────────────────────────────────────────────────

    def _cmd_start_data_upload_cb(self, _msg: Empty):
        """Trigger background upload of the current sweep data."""
        t = threading.Thread(target=self._upload_worker, daemon=True)
        t.start()

    def _upload_worker(self):
        self._publish_upload_status("UPLOADING")
        gd = self.cfg.get('google_drive', {})
        if not gd.get('enabled', False):
            self.get_logger().warn('[upload] Google Drive upload is not enabled in config.')
            self._publish_upload_status("FAILED", error="Google Drive upload is not enabled in config")
            return

        remote             = gd.get('rclone_remote', 'gdrive')
        base_remote_folder = gd.get('remote_folder', 'sweep_test_data')

        try:
            rel_path      = os.path.relpath(self._current_sweep_dir, self._output_dir)
            remote_folder = os.path.join(base_remote_folder, rel_path).replace('\\', '/')
        except ValueError:
            remote_folder = base_remote_folder

        self.get_logger().info(
            f'[upload] Starting upload of {self._current_sweep_dir} to {remote}:{remote_folder}')

        ok = rclone_upload(self._current_sweep_dir, remote, remote_folder, self.get_logger())

        if not ok:
            self.get_logger().warn('[upload] rclone upload failed.')
            self._publish_upload_status("FAILED", error="rclone upload failed")
            self._gdrive_ok = False
        else:
            self.get_logger().info(f'[upload] ✓ Successfully uploaded {self._current_sweep_dir}')
            self._publish_upload_status("COMPLETED")
            self._gdrive_ok = True

    def _publish_upload_status(self, status: str, error: str = ""):
        msg = json.dumps({"status": status, "error": error})
        self._pub_upload_status.publish(String(data=msg))

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _capture_remaining(self) -> float:
        if self._capture_start_ns is None:
            return 0.0
        elapsed = (self.get_clock().now().nanoseconds - self._capture_start_ns) / 1e9
        return max(0.0, self._capture_duration - elapsed)

    def _count_captures_in_folder(self) -> int:
        """Count finished captures (CSV files) in the current sweep folder."""
        try:
            return sum(
                1 for f in os.listdir(self._current_sweep_dir)
                if f.endswith('.csv')
            )
        except Exception:
            return 0

    def _publish_placement(self):
        """Publish the target placement instruction for the current step."""
        with self._state_lock:
            state = self._state
            steps = self._steps
            idx   = self._step_index

        if not steps or state in (STATE_IDLE, STATE_DONE):
            instruction = (
                '[SWEEP TEST]  No scenario configured.\n'
                'Send a configure message:\n'
                '  ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \\\n'
                '    \'data: "{\\\"mode\\\":\\\"outdoor\\\",\\\"scenario\\\":\\\"custom\\\","'
                '\\\"sensors\\\":\\\"both\\\",\\\"distance\\\":10.0,\\\"angle\\\":0.0}"\''
            ) if state == STATE_IDLE else \
                '[SWEEP TEST]  All steps complete! Send /configure to run again.'
        elif idx >= len(steps):
            instruction = '[SWEEP TEST]  All steps complete!'
        else:
            instruction = steps[idx]['instruction']

        self._pub_placement.publish(String(data=instruction))
        self.get_logger().info(f'[placement] {instruction}')

    def _publish_capturing(self, value: bool):
        self._pub_capturing.publish(Bool(data=value))

    def _status_timer_cb(self):
        """Publish JSON status at 2 Hz."""
        now_ns = self.get_clock().now().nanoseconds

        with self._state_lock:
            state     = self._state
            step_idx  = self._step_index
            steps     = self._steps
            mode      = self._mode
            scenario  = self._scenario
            sensors   = self._sensors
            distance  = self._distance
            angle     = self._angle
            last_csv  = self._last_csv
            gdrive_ok = self._gdrive_ok

        capturing = (state == STATE_CAPTURING)
        if capturing and self._capture_start_ns:
            elapsed_s = (now_ns - self._capture_start_ns) / 1e9
        else:
            elapsed_s = 0.0

        step_label = ''
        step_total = len(steps)
        if steps and step_idx < step_total:
            step_label = steps[step_idx]['label']

        if not steps or state in (STATE_IDLE, STATE_DONE):
            instruction = (
                'No scenario configured. Send configure message to begin.'
            ) if state == STATE_IDLE else \
                'All steps complete! Send configure message to run again.'
        elif state == STATE_STARTING_SENSORS:
            instruction = (
                f'Auto-starting sensors, waiting {self._sensor_startup_grace_s:.0f}s '
                f'grace period before capture...'
            )
        elif step_idx >= step_total:
            instruction = 'All steps complete!'
        else:
            instruction = steps[step_idx]['instruction']

        status = {
            'state':              state,
            'instruction':        instruction,
            'captures_in_folder': self._count_captures_in_folder(),
            'samples_this_step':  self._recorder.sample_count,
            'images_this_step':       self._recorder.image_count,
            'pointclouds_this_step':  self._recorder.pointcloud_count,
            'capturing':          capturing,
            'capture_elapsed_s':  round(elapsed_s, 2),
            'distance':           distance,
            'angle':              angle,
            'step_index':         step_idx,
            'step_total':         step_total,
            'mode':               mode,
            'scenario':           scenario,
            'sensors':            sensors,
            'step_label':         step_label,
            'capture_duration_s': self._capture_duration,
            'last_file':          last_csv,
            'gdrive_upload_ok':   gdrive_ok,
            'timestamp_ns':       now_ns,
        }

        self._pub_status.publish(String(data=json.dumps(status)))
        self._publish_capturing(capturing)


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SensorSweepNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node._recorder.is_recording:
            node._recorder.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
