"""Test D: Environmental Stress Tests (Edge Cases).

Procedure
---------
Repeat the Test-C dynamic approach under degraded conditions:

*Simulation*: modify Gazebo ``<ambient>`` light to simulate Sun / Cloudy /
Night.  Add Gaussian noise to the camera plugin for Rain / Sensor degradation.

*Real world*: test outdoors or manipulate indoor lighting (lights on/off,
blinding spotlight).

Each condition runs a full Test-C dynamic approach with the condition label
tagged in the CSV.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from .config_loader import load_config
from .data_recorder import DataRecorder
from .robot_driver import RobotDriver
from .gdrive_uploader import maybe_upload


class TestDEnvironmental(Node):
    def __init__(self):
        super().__init__('test_d_environmental')
        self.declare_parameter('config_path', '')
        cp = self.get_parameter('config_path').get_parameter_value().string_value
        self.cfg = load_config(cp if cp else None)
        self.recorder = DataRecorder(self, self.cfg)
        self.driver = RobotDriver(self, self.cfg)

        self._conditions = self.cfg['test_d']['conditions']

        # Test C parameters (inherited)
        self._start_dist = self.cfg['test_c']['start_distance']
        self._speed = self.cfg['test_c']['approach_speed']
        self._timeout = self.cfg['test_c']['timeout']
        self._stop_dist = self.cfg['test_c'].get('stop_distance', 0.15)

        # Offboard heartbeat
        self._hb_timer = self.create_timer(0.05, self.driver.send_offboard_heartbeat)

    def _run_single_approach(self, condition: str) -> str:
        """Run one Test-C-style approach under a given condition."""
        import math
        from geometry_msgs.msg import Twist

        test_name = f'test_d_{condition}'
        label = f'{condition}_approach_{self._speed:.1f}mps'

        self.recorder.start_recording(test_name, label)

        # Arm and offboard
        print('  Arming and enabling offboard mode...')
        self.driver.set_offboard(True)
        time.sleep(0.5)
        self.driver.arm()
        time.sleep(1.0)

        msg = Twist()
        msg.linear.x = self._speed

        start_time = time.monotonic()
        stopped = False

        while time.monotonic() - start_time < self._timeout:
            elapsed = time.monotonic() - start_time
            count = self.recorder.sample_count
            cam_dist = self.recorder.latest_cam_distance

            if not math.isnan(cam_dist) and cam_dist < self._stop_dist:
                print(f'\n    Tag distance {cam_dist:.3f}m < '
                      f'{self._stop_dist}m - stopping.')
                stopped = True
                break

            self.driver._cmd_pub.publish(msg)
            print(f'\r    {elapsed:5.1f}s  |  samples: {count}  |  '
                  f'tag dist: {cam_dist:.3f}m', end='', flush=True)
            time.sleep(1.0 / self.driver.control_rate)

        self.driver.stop()
        print()

        if not stopped:
            print(f'    Timeout reached ({self._timeout}s).')

        csv_path = self.recorder.stop_recording()
        self.driver.disarm()
        return csv_path

    def run_test(self):
        print('\n' + '=' * 60)
        print('  TEST D: Environmental Stress Tests')
        print('=' * 60)
        print(f'  Conditions: {self._conditions}')
        print(f'  Each condition runs a Test-C approach:')
        print(f'    Start: {self._start_dist} m, Speed: {self._speed} m/s')
        print('=' * 60 + '\n')

        csv_paths = []

        for i, condition in enumerate(self._conditions):
            print(f'\n{"="*50}')
            print(f'  Condition {i + 1}/{len(self._conditions)}: {condition.upper()}')
            print(f'{"="*50}')

            # Instructions for each condition
            instructions = {
                'sun': 'Set Gazebo ambient light to bright sun, OR test in direct sunlight.',
                'cloudy': 'Set Gazebo ambient light to overcast, OR test under cloud cover.',
                'night': 'Set Gazebo ambient light to near-dark, OR turn off all lights.',
                'rain_sim': 'Enable Gaussian noise plugin on the camera (simulate rain/fog).',
                'lights_on': 'Ensure indoor lights are ON (normal conditions).',
                'lights_off': 'Turn OFF all indoor lights (dark environment).',
                'spotlight': 'Aim a bright spotlight directly at the camera (glare test).',
            }
            instr = instructions.get(condition,
                                     f'Set up the "{condition}" condition as appropriate.')
            print(f'\n  Instructions: {instr}')
            print(f'\n  Place the robot {self._start_dist} m from the docking station, '
                  f'facing it.')
            input('  Press ENTER when the condition is set and robot is positioned...\n')

            csv_path = self._run_single_approach(condition)
            csv_paths.append(csv_path)
            print(f'  Saved: {csv_path}')

            maybe_upload(self.cfg, csv_path, self.get_logger())

            if i < len(self._conditions) - 1:
                print('\n  Prepare for the next condition.')
                input('  Press ENTER to continue...')

        print('\n' + '=' * 60)
        print('  TEST D COMPLETE')
        print('=' * 60)
        print('  All condition CSVs:')
        for p in csv_paths:
            print(f'    {p}')
        print('\nYou may now press Ctrl+C to exit.')


def main(args=None):
    rclpy.init(args=args)
    node = TestDEnvironmental()

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
