"""Sensor Sweep Test Node — topic-driven, no robot driving.

State machine
─────────────
  IDLE
    │  /sweep_test/cmd/configure (JSON)
    ▼
  CONFIGURED
    │  /sweep_test/cmd/capture (Empty)
    ▼
  CAPTURING  ── (abort) ──► CONFIGURED
    │  (10 s elapsed)
    ▼
  UPLOADING
    │  (rclone done)
    ▼
  CONFIGURED  (next step)  or  DONE  (all steps finished)
    │  /sweep_test/cmd/configure to restart
    ▼
  CONFIGURED

Published topics
────────────────
  /sweep_test/status           String (JSON)  2 Hz
  /sweep_test/target_placement String          on change
  /sweep_test/capturing        Bool            continuously
  /sweep_test/step_complete    String (JSON)   once per step

Subscribed topics (trigger / command)
──────────────────────────────────────
  /sweep_test/cmd/configure    String (JSON)
  /sweep_test/cmd/capture      Empty
  /sweep_test/cmd/abort        Empty

Usage (from shell)
──────────────────
  ros2 launch docking_test_suite sensor_sweep.launch.py

  # Configure a distance sweep, outdoor, both sensors
  ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \\
    'data: "{\"mode\":\"outdoor\",\"scenario\":\"distance\",\"sensors\":\"both\"}"'

  # Watch state + placement instructions
  ros2 topic echo /sweep_test/status
  ros2 topic echo /sweep_test/target_placement

  # Trigger capture when target is in place
  ros2 topic pub --once /sweep_test/cmd/capture std_msgs/Empty '{}'

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

from std_msgs.msg import String, Bool, Empty

from .config_loader import load_config, get_output_dir
from .sweep_recorder import SweepRecorder, rclone_upload
from .sweep_scenarios import distance_sweep_steps, angular_sweep_steps
from .gdrive_uploader import maybe_upload   # OAuth2 fallback


# ── State constants ──────────────────────────────────────────────────────────

STATE_IDLE       = 'IDLE'
STATE_CONFIGURED = 'CONFIGURED'
STATE_CAPTURING  = 'CAPTURING'
STATE_UPLOADING  = 'UPLOADING'
STATE_DONE       = 'DONE'


class SensorSweepNode(Node):
    """Topic-driven sensor sweep test node."""

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

        # ── State ────────────────────────────────────────────────────────
        self._state = STATE_IDLE
        self._mode = ''
        self._scenario = ''
        self._sensors = 'both'
        self._steps = []
        self._step_index = 0
        self._capture_start_ns = None
        self._last_csv = ''
        self._last_bag = ''
        self._gdrive_ok = None   # None = not attempted, True/False = result
        self._state_lock = threading.Lock()

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

        # ── Subscribers (trigger topics) ─────────────────────────────────
        self.create_subscription(
            String, '/sweep_test/cmd/configure',
            self._cmd_configure_cb, 10)
        self.create_subscription(
            Empty, '/sweep_test/cmd/capture',
            self._cmd_capture_cb, 10)
        self.create_subscription(
            Empty, '/sweep_test/cmd/abort',
            self._cmd_abort_cb, 10)

        # ── Status heartbeat timer (2 Hz) ────────────────────────────────
        self._status_timer = self.create_timer(0.5, self._status_timer_cb)

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════╗\n'
            '║         SENSOR SWEEP TEST NODE  —  READY             ║\n'
            '╠══════════════════════════════════════════════════════╣\n'
            '║  Configure:  /sweep_test/cmd/configure  (String JSON) ║\n'
            '║  Capture:    /sweep_test/cmd/capture    (Empty)       ║\n'
            '║  Abort:      /sweep_test/cmd/abort      (Empty)       ║\n'
            '║  Status:     /sweep_test/status         (echo)        ║\n'
            '║  Placement:  /sweep_test/target_placement (echo)      ║\n'
            '╚══════════════════════════════════════════════════════╝\n'
            '\nSend a configure message to begin:\n'
            '  ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \\\n'
            '    \'data: "{\\\"mode\\\":\\\"outdoor\\\","'
            '\\\"scenario\\\":\\\"distance\\\",\\\"sensors\\\":\\\"both\\\"}"\''
        )

    # ── Command callbacks ────────────────────────────────────────────────────

    def _cmd_configure_cb(self, msg: String):
        """Parse JSON configure message and transition to CONFIGURED."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'[configure] Invalid JSON: {e}\n'
                f'  Expected: {{"mode":"outdoor","scenario":"distance","sensors":"both"}}')
            return

        mode = data.get('mode', 'outdoor')
        scenario = data.get('scenario', 'distance')
        sensors = data.get('sensors', 'both')
        step_index = int(data.get('step_index', 0))

        # Validate
        if mode not in ('indoor', 'outdoor'):
            self.get_logger().error(
                f'[configure] Invalid mode "{mode}". Use "indoor" or "outdoor".')
            return
        if scenario not in ('distance', 'angular'):
            self.get_logger().error(
                f'[configure] Invalid scenario "{scenario}". Use "distance" or "angular".')
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

            self._mode = mode
            self._scenario = scenario
            self._sensors = sensors
            self._recorder.sensors = sensors
            self._gdrive_ok = None

            # Build step list
            sweep_cfg = self.cfg.get('sweep_test', {})
            if scenario == 'distance':
                ds = sweep_cfg.get('distance_sweep', {})
                self._steps = distance_sweep_steps(
                    start_m=float(ds.get('start_m', 10.0)),
                    stop_m=float(ds.get('stop_m', 1.0)),
                    step_m=float(ds.get('step_m', 1.0)),
                )
            else:
                as_ = sweep_cfg.get('angular_sweep', {})
                self._steps = angular_sweep_steps(
                    fixed_dist_m=float(as_.get('fixed_distance_m', 3.0)),
                    start_deg=int(as_.get('start_deg', 0)),
                    stop_deg=int(as_.get('stop_deg', 180)),
                    step_deg=int(as_.get('step_deg', 20)),
                )

            self._step_index = max(0, min(step_index, len(self._steps) - 1))
            self._state = STATE_CONFIGURED
            self._last_csv = ''
            self._last_bag = ''

        self.get_logger().info(
            f'[configure] ✓ mode={mode}  scenario={scenario}  '
            f'sensors={sensors}  steps={len(self._steps)}  '
            f'starting at step {self._step_index + 1}')

        self._publish_placement()

    def _cmd_capture_cb(self, _msg: Empty):
        """Trigger a 10 s capture of the current step."""
        with self._state_lock:
            if self._state == STATE_CAPTURING:
                remaining = self._capture_remaining()
                self.get_logger().warn(
                    f'[capture] Already capturing — {remaining:.1f} s remaining.')
                return
            if self._state not in (STATE_CONFIGURED,):
                self.get_logger().warn(
                    f'[capture] Cannot capture in state {self._state}. '
                    f'Send /sweep_test/cmd/configure first.')
                return
            if not self._steps:
                self.get_logger().error('[capture] No steps defined — configure first.')
                return

            self._state = STATE_CAPTURING
            self._capture_start_ns = self.node.get_clock().now().nanoseconds

        step = self._steps[self._step_index]
        self.get_logger().info(
            f'\n{"═"*60}\n'
            f'  CAPTURING  step {self._step_index + 1}/{len(self._steps)}\n'
            f'  label:    {step["label"]}\n'
            f'  mode:     {self._mode}  |  sensors: {self._sensors}\n'
            f'  duration: {self._capture_duration:.0f} s\n'
            f'{"═"*60}'
        )

        # Publish Bool = True immediately
        self._publish_capturing(True)

        # Start recorder
        self._recorder.start(
            step_label=step['label'],
            scenario=self._scenario,
            mode=self._mode,
            output_dir=self._output_dir,
        )

        # Spawn capture timer in background
        t = threading.Thread(target=self._capture_worker, daemon=True)
        t.start()

    def _cmd_abort_cb(self, _msg: Empty):
        """Abort an ongoing capture without saving."""
        with self._state_lock:
            if self._state != STATE_CAPTURING:
                self.get_logger().info(
                    f'[abort] Nothing to abort (state={self._state})')
                return
            self._state = STATE_CONFIGURED

        self._recorder.abort()
        self._publish_capturing(False)
        self.get_logger().warn('[abort] ✗ Capture ABORTED — data discarded.')
        self._publish_placement()

    # ── Capture worker ───────────────────────────────────────────────────────

    def _capture_worker(self):
        """Background thread: wait capture_duration, then finalize."""
        interval = 1.0
        elapsed = 0.0
        while elapsed < self._capture_duration:
            time.sleep(min(interval, self._capture_duration - elapsed))
            elapsed += interval

            # Check for abort
            with self._state_lock:
                if self._state != STATE_CAPTURING:
                    return   # aborted

            # Heartbeat log every 2 s
            if int(elapsed) % 2 == 0:
                self.get_logger().info(
                    f'  ⏱  {elapsed:.0f}/{self._capture_duration:.0f} s  '
                    f'| {self._recorder.sample_count} samples')

        # Capture window complete
        self._finalize_capture()

    def _finalize_capture(self):
        """Stop recorder, write files, upload, advance step."""
        with self._state_lock:
            if self._state != STATE_CAPTURING:
                return
            self._state = STATE_UPLOADING

        self._publish_capturing(False)

        csv_path, bag_dir = self._recorder.stop()
        n = self._recorder.sample_count  # already 0 after stop, store before
        step = self._steps[self._step_index]

        # Read actual count from file
        try:
            import csv as _csv
            with open(csv_path, 'r') as f:
                n = sum(1 for _ in f) - 1  # minus header
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

        # Publish step_complete
        step_done_msg = json.dumps({
            'step_label':  step['label'],
            'samples':     n,
            'csv_file':    csv_path,
            'bag_dir':     bag_dir,
            'duration_s':  self._capture_duration,
        })
        self._pub_step_done.publish(String(data=step_done_msg))

        # ── Upload ───────────────────────────────────────────────────────
        gd = self.cfg.get('google_drive', {})
        upload_ok = None
        if gd.get('enabled', False) and gd.get('auto_upload', False):
            remote = gd.get('rclone_remote', 'gdrive')
            folder = gd.get('remote_folder', 'sweep_test_data')
            # Upload CSV
            ok1 = rclone_upload(csv_path, remote, folder, self.get_logger())
            # Upload bag directory
            ok2 = rclone_upload(bag_dir, remote, folder, self.get_logger()) \
                if os.path.isdir(bag_dir) else True
            upload_ok = ok1 and ok2

            # Fallback to OAuth2 Python API if rclone failed
            if not upload_ok:
                self.get_logger().warn(
                    '[upload] rclone failed — trying OAuth2 Google Drive API fallback')
                try:
                    maybe_upload(self.cfg, csv_path, self.get_logger())
                    upload_ok = True
                except Exception as e:
                    self.get_logger().error(f'[upload] OAuth2 fallback also failed: {e}')

        self._gdrive_ok = upload_ok

        # ── Advance step ─────────────────────────────────────────────────
        with self._state_lock:
            self._step_index += 1
            if self._step_index >= len(self._steps):
                self._state = STATE_DONE
                self.get_logger().info(
                    '\n╔══════════════════════════════════════╗\n'
                    '║   ALL STEPS COMPLETE — sweep DONE!   ║\n'
                    '║   Send /configure to restart.         ║\n'
                    '╚══════════════════════════════════════╝')
            else:
                self._state = STATE_CONFIGURED

        self._publish_placement()

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _capture_remaining(self) -> float:
        if self._capture_start_ns is None:
            return 0.0
        elapsed = (self.node.get_clock().now().nanoseconds - self._capture_start_ns) / 1e9
        return max(0.0, self._capture_duration - elapsed)

    def _publish_placement(self):
        """Publish the target placement instruction for the current step."""
        with self._state_lock:
            state = self._state
            steps = self._steps
            idx = self._step_index

        if not steps or state in (STATE_IDLE, STATE_DONE):
            instruction = (
                '[SWEEP TEST]  No scenario configured.\n'
                'Send a configure message:\n'
                '  ros2 topic pub --once /sweep_test/cmd/configure std_msgs/String \\\n'
                '    \'data: "{\\\"mode\\\":\\\"outdoor\\\","'
                '\\\"scenario\\\":\\\"distance\\\",\\\"sensors\\\":\\\"both\\\"}"\''
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
            state = self._state
            step_idx = self._step_index
            steps = self._steps
            mode = self._mode
            scenario = self._scenario
            sensors = self._sensors
            last_csv = self._last_csv
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

        status = {
            'state':              state,
            'mode':               mode,
            'scenario':           scenario,
            'sensors':            sensors,
            'step_index':         step_idx,
            'step_total':         step_total,
            'step_label':         step_label,
            'capturing':          capturing,
            'capture_elapsed_s':  round(elapsed_s, 2),
            'capture_duration_s': self._capture_duration,
            'samples_this_step':  self._recorder.sample_count,
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
