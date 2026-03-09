"""Test B: Static Angular Profile.

Procedure
---------
Place the robot at a fixed distance (default 3 m).  Rotate the robot / camera
in 10° increments from 0° to 60° left and right.  Record 10 s of data at each
angle.

Goal
----
Determine orientation accuracy and the maximum viewing-angle limit.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .config_loader import load_config
from .data_recorder import DataRecorder
from .gdrive_uploader import maybe_upload


class TestBAngular(Node):
    def __init__(self):
        super().__init__('test_b_angular')
        self.declare_parameter('config_path', '')
        cp = self.get_parameter('config_path').get_parameter_value().string_value
        self.cfg = load_config(cp if cp else None)
        self.recorder = DataRecorder(self, self.cfg)

        self._distance = self.cfg['test_b']['fixed_distance']
        self._angles = self.cfg['test_b']['angles']
        self._duration = self.cfg['test_b']['record_duration']

    def run_test(self):
        test_name = 'test_b_angular'
        print('\n' + '=' * 60)
        print('  TEST B: Static Angular Profile')
        print('=' * 60)
        print(f'  Fixed distance: {self._distance} m')
        print(f'  Angles: {self._angles}')
        print(f'  Record duration per step: {self._duration}s')
        print('=' * 60 + '\n')

        print(f'Place the robot at {self._distance} m from the docking station.')
        input('Press ENTER to begin...\n')

        self.recorder.start_recording(test_name, '')

        for i, angle in enumerate(self._angles):
            if angle < 0:
                direction = f'{abs(angle)}° RIGHT'
            elif angle > 0:
                direction = f'{angle}° LEFT'
            else:
                direction = '0° (straight)'

            label = f'angle_{angle:+d}deg'
            print(f'\n--- Step {i + 1}/{len(self._angles)}: {direction} ---')
            print(f'Rotate the robot/camera to {direction} at {self._distance} m.')
            input('Press ENTER when ready to record...')

            self.recorder.update_label(label)
            print(f'Recording {self._duration}s ...')

            start = time.monotonic()
            while time.monotonic() - start < self._duration:
                elapsed = time.monotonic() - start
                count = self.recorder.sample_count
                print(f'\r  {elapsed:5.1f}s / {self._duration:.0f}s  |  '
                      f'{count} samples', end='', flush=True)
                time.sleep(0.5)
            print()

            count = self.recorder.sample_count
            print(f'  Recorded. Total samples so far: {count}')

        csv_path = self.recorder.stop_recording()
        print(f'\nTest B complete. Data saved to:\n  {csv_path}')

        maybe_upload(self.cfg, csv_path, self.get_logger())

        print('\nYou may now press Ctrl+C to exit.')


def main(args=None):
    rclpy.init(args=args)
    node = TestBAngular()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    test_thread = threading.Thread(target=node.run_test, daemon=True)
    test_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.recorder.is_recording:
            node.recorder.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
