"""Test C: Dynamic Approach (Everyday Scenario).

Procedure
---------
Start the robot 5 m out.  Drive straight into the dock at a constant velocity
(default 0.2 m/s).  Record a continuous data stream from start to dock.

Goal
----
Measure dynamic tracking stability and detect "pose jumping" (discontinuities
in the transform).
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .config_loader import load_config
from .data_recorder import DataRecorder
from .robot_driver import RobotDriver
from .gdrive_uploader import maybe_upload


class TestCDynamic(Node):
    def __init__(self):
        super().__init__('test_c_dynamic')
        self.declare_parameter('config_path', '')
        self.declare_parameter('condition_label', '')
        cp = self.get_parameter('config_path').get_parameter_value().string_value
        self.cfg = load_config(cp if cp else None)
        self.recorder = DataRecorder(self, self.cfg)
        self.driver = RobotDriver(self, self.cfg)

        self._start_dist = self.cfg['test_c']['start_distance']
        self._speed = self.cfg['test_c']['approach_speed']
        self._timeout = self.cfg['test_c']['timeout']
        self._stop_dist = self.cfg['test_c'].get('stop_distance', 0.15)

        # Offboard heartbeat
        self._hb_timer = self.create_timer(0.05, self.driver.send_offboard_heartbeat)

    def run_test(self, condition_label: str = ''):
        """Run a single dynamic approach.

        Parameters
        ----------
        condition_label : str
            Optional label for Test-D conditions (e.g. 'sun', 'night').
        """
        if not condition_label:
            condition_label = self.get_parameter(
                'condition_label').get_parameter_value().string_value

        test_name = 'test_c_dynamic'
        if condition_label:
            test_name = f'test_d_{condition_label}'

        print('\n' + '=' * 60)
        print('  TEST C: Dynamic Approach')
        if condition_label:
            print(f'  Condition: {condition_label}')
        print('=' * 60)
        print(f'  Start distance: {self._start_dist} m')
        print(f'  Approach speed: {self._speed} m/s')
        print(f'  Timeout: {self._timeout} s')
        print(f'  Stop distance: {self._stop_dist} m')
        print('=' * 60 + '\n')

        print(f'Place the robot {self._start_dist} m in front of the docking station,')
        print('facing it directly.')
        input('Press ENTER when ready to start the approach...\n')

        # Start recording
        label = f'approach_{self._speed:.1f}mps'
        if condition_label:
            label = f'{condition_label}_{label}'
        self.recorder.start_recording(test_name, label)

        # Arm and offboard
        print('Arming and enabling offboard mode...')
        self.driver.set_offboard(True)
        time.sleep(0.5)
        self.driver.arm()
        time.sleep(1.0)

        # Drive forward at constant speed
        print(f'Driving forward at {self._speed} m/s ...')
        expected_duration = self._start_dist / max(self._speed, 0.01)

        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x = self._speed

        start_time = time.monotonic()
        stopped = False

        while time.monotonic() - start_time < self._timeout:
            elapsed = time.monotonic() - start_time
            count = self.recorder.sample_count
            cam_dist = self.recorder.latest_cam_distance

            # Check if close enough to stop
            if not math.isnan(cam_dist) and cam_dist < self._stop_dist:
                print(f'\n  Tag distance {cam_dist:.3f}m < {self._stop_dist}m - stopping.')
                stopped = True
                break

            self.driver._cmd_pub.publish(msg)
            print(f'\r  {elapsed:5.1f}s  |  samples: {count}  |  '
                  f'tag dist: {cam_dist:.3f}m', end='', flush=True)
            time.sleep(1.0 / self.driver.control_rate)

        self.driver.stop()
        print()

        if not stopped:
            print(f'  Timeout reached ({self._timeout}s).')

        csv_path = self.recorder.stop_recording()
        print(f'\nTest complete. Data saved to:\n  {csv_path}')

        maybe_upload(self.cfg, csv_path, self.get_logger())

        # Disarm
        self.driver.disarm()
        print('\nYou may now press Ctrl+C to exit.')

        return csv_path


def main(args=None):
    rclpy.init(args=args)
    node = TestCDynamic()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    test_thread = threading.Thread(target=node.run_test, daemon=True)
    test_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.stop()
        if node.recorder.is_recording:
            node.recorder.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
