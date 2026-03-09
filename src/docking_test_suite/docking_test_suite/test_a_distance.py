"""Test A: Static Distance Profile.

Procedure
---------
Place the robot directly facing the docking station.  Start at 10 m and move
closer in 1 m increments until docked (0 m).  Record 10 s of data at each
interval.

Goal
----
Determine maximum / minimum detection range and distance-dependent accuracy
degradation.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .config_loader import load_config
from .data_recorder import DataRecorder
from .gdrive_uploader import maybe_upload


class TestADistance(Node):
    def __init__(self):
        super().__init__('test_a_distance')
        self.declare_parameter('config_path', '')
        cp = self.get_parameter('config_path').get_parameter_value().string_value
        self.cfg = load_config(cp if cp else None)
        self.recorder = DataRecorder(self, self.cfg)

        self._distances = self.cfg['test_a']['distances']
        self._duration = self.cfg['test_a']['record_duration']

    def run_test(self):
        test_name = 'test_a_distance'
        print('\n' + '=' * 60)
        print('  TEST A: Static Distance Profile')
        print('=' * 60)
        print(f'  Distances: {self._distances}')
        print(f'  Record duration per step: {self._duration}s')
        print('=' * 60 + '\n')

        self.recorder.start_recording(test_name, '')

        for i, dist in enumerate(self._distances):
            label = f'distance_{dist:.1f}m'
            print(f'\n--- Step {i + 1}/{len(self._distances)}: {dist:.1f} m ---')
            print(f'Place the robot at {dist:.1f} m facing the docking station.')

            if dist == 0.0:
                print('(Docked position)')

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
        print(f'\nTest A complete. Data saved to:\n  {csv_path}')

        maybe_upload(self.cfg, csv_path, self.get_logger())

        print('\nYou may now press Ctrl+C to exit.')


def main(args=None):
    rclpy.init(args=args)
    node = TestADistance()

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
