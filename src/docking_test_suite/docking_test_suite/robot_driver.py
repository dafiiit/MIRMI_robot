"""Robot driving utilities and interactive debug CLI.

Provides both programmatic and interactive control for:
- Drive exact distance at exact speed (open-loop & closed-loop)
- Turn exact angle at exact speed
- Emergency stop
- Arm / disarm / offboard mode
- Live position display
- System diagnostics
"""

import math
import time
import threading
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped

from .config_loader import load_config
from .transforms import yaw_from_quaternion

try:
    from px4_msgs.msg import VehicleCommand, OffboardControlMode, VehicleOdometry
    _HAS_PX4 = True
except ImportError:
    _HAS_PX4 = False


class RobotDriver:
    """Attach to a ROS 2 Node and provide precise robot movement commands.

    Supports both:
    - **Open-loop**: time-based (``drive_duration = distance / speed``)
    - **Closed-loop**: feedback from Vicon or PX4 odometry
    """

    def __init__(self, node: Node, config: dict):
        self.node = node
        self.cfg = config
        self.topics = config['topics']
        drv = config['system']['robot_driver']
        self.max_lin = drv['max_linear_speed']
        self.max_ang = drv['max_angular_speed']
        self.control_rate = drv['control_rate']
        self.pos_tol = drv['position_tolerance']
        self.ang_tol = math.radians(drv['angle_tolerance'])
        self.use_vicon = config['system']['use_vicon']

        self._dt = 1.0 / self.control_rate

        # Publishers
        self._cmd_pub = self.node.create_publisher(
            Twist, self.topics['cmd_vel'], 10)

        if _HAS_PX4:
            qos_px4 = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=10)
            self._cmd_px4_pub = self.node.create_publisher(
                VehicleCommand, self.topics['vehicle_command'], qos_px4)
            self._offboard_pub = self.node.create_publisher(
                OffboardControlMode, self.topics['offboard_control_mode'], qos_px4)

        # Position feedback
        self._current_position = None   # [x, y, z]
        self._current_yaw = None        # radians
        self._pose_lock = threading.Lock()

        if self.use_vicon:
            self.node.create_subscription(
                PoseStamped, self.topics['vicon_robot_pose'],
                self._vicon_cb, 10)
        elif _HAS_PX4:
            qos_odom = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=1)
            self.node.create_subscription(
                VehicleOdometry,
                self.topics.get('px4_odometry', '/fmu/out/vehicle_odometry'),
                self._px4_odom_cb, qos_odom)

    # ------------------------------------------------------------------
    # Pose callbacks
    # ------------------------------------------------------------------

    def _vicon_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        yaw_deg = yaw_from_quaternion([o.x, o.y, o.z, o.w])
        with self._pose_lock:
            self._current_position = np.array([p.x, p.y, p.z])
            self._current_yaw = math.radians(yaw_deg)

    def _px4_odom_cb(self, msg):
        with self._pose_lock:
            self._current_position = np.array(list(msg.position))
            q = msg.q  # [w, x, y, z] in PX4
            yaw_deg = yaw_from_quaternion([q[1], q[2], q[3], q[0]])
            self._current_yaw = math.radians(yaw_deg)

    def get_position(self):
        """Return current [x, y, z] or None."""
        with self._pose_lock:
            if self._current_position is not None:
                return self._current_position.copy()
            return None

    def get_yaw(self):
        """Return current yaw in radians or None."""
        with self._pose_lock:
            return self._current_yaw

    # ------------------------------------------------------------------
    # PX4 commands
    # ------------------------------------------------------------------

    def arm(self):
        """Send ARM command to PX4."""
        if not _HAS_PX4:
            self.node.get_logger().error('px4_msgs not available')
            return
        t_us = self.node.get_clock().now().nanoseconds // 1000
        msg = VehicleCommand()
        msg.timestamp = t_us
        msg.command = 400  # VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_px4_pub.publish(msg)
        self.node.get_logger().info('ARM command sent')

    def disarm(self):
        """Send DISARM command to PX4."""
        if not _HAS_PX4:
            return
        t_us = self.node.get_clock().now().nanoseconds // 1000
        msg = VehicleCommand()
        msg.timestamp = t_us
        msg.command = 400
        msg.param1 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_px4_pub.publish(msg)
        self.node.get_logger().info('DISARM command sent')

    def set_offboard(self, enable: bool = True):
        """Send OFFBOARD or POSCTL mode command."""
        if not _HAS_PX4:
            return
        t_us = self.node.get_clock().now().nanoseconds // 1000
        msg = VehicleCommand()
        msg.timestamp = t_us
        msg.command = 176  # VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0 if enable else 3.0  # OFFBOARD or POSCTL
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_px4_pub.publish(msg)
        self.node.get_logger().info(
            'OFFBOARD mode enabled' if enable else 'POSCTL mode enabled')

    def send_offboard_heartbeat(self):
        """Publish one OffboardControlMode heartbeat."""
        if not _HAS_PX4:
            return
        t_us = self.node.get_clock().now().nanoseconds // 1000
        m = OffboardControlMode()
        m.timestamp = t_us
        m.position = False
        m.velocity = True
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        if hasattr(m, 'direct_actuator'):
            m.direct_actuator = False
        self._offboard_pub.publish(m)

    # ------------------------------------------------------------------
    # Motion commands
    # ------------------------------------------------------------------

    def stop(self):
        """Immediately publish zero velocity."""
        msg = Twist()
        self._cmd_pub.publish(msg)

    def drive_distance(self, distance_m: float, speed_mps: float,
                       closed_loop: bool = False) -> bool:
        """Drive forward (positive) or backward (negative) by *distance_m*.

        Parameters
        ----------
        distance_m : float
            Signed distance. Positive = forward, negative = backward.
        speed_mps : float
            Absolute speed (always positive).
        closed_loop : bool
            If True and position feedback is available, use it.

        Returns True if completed successfully.
        """
        speed_mps = min(abs(speed_mps), self.max_lin)
        direction = 1.0 if distance_m >= 0 else -1.0
        distance_m = abs(distance_m)

        if closed_loop and self.get_position() is not None:
            return self._drive_closed_loop(distance_m, speed_mps, direction)
        else:
            return self._drive_open_loop(distance_m, speed_mps, direction)

    def _drive_open_loop(self, distance_m, speed_mps, direction):
        duration = distance_m / max(speed_mps, 1e-6)
        self.node.get_logger().info(
            f'Driving {"forward" if direction > 0 else "backward"} '
            f'{distance_m:.2f}m at {speed_mps:.2f} m/s '
            f'(open-loop, {duration:.1f}s)')

        msg = Twist()
        msg.linear.x = direction * speed_mps
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.send_offboard_heartbeat()
            self._cmd_pub.publish(msg)
            time.sleep(self._dt)

        self.stop()
        self.node.get_logger().info('Drive complete')
        return True

    def _drive_closed_loop(self, distance_m, speed_mps, direction):
        start_pos = self.get_position()
        if start_pos is None:
            self.node.get_logger().warn('No position feedback, falling back to open-loop')
            return self._drive_open_loop(distance_m, speed_mps, direction)

        self.node.get_logger().info(
            f'Driving {"forward" if direction > 0 else "backward"} '
            f'{distance_m:.2f}m at {speed_mps:.2f} m/s (closed-loop)')

        timeout = (distance_m / max(speed_mps, 1e-6)) * 3.0 + 5.0
        start_time = time.monotonic()
        msg = Twist()

        while True:
            cur = self.get_position()
            if cur is not None:
                traveled = np.linalg.norm(cur[:2] - start_pos[:2])
                remaining = distance_m - traveled
                if remaining <= self.pos_tol:
                    break
                v = min(speed_mps, max(0.05, remaining))
            else:
                v = speed_mps

            msg.linear.x = direction * v
            self.send_offboard_heartbeat()
            self._cmd_pub.publish(msg)

            if time.monotonic() - start_time > timeout:
                self.node.get_logger().warn('Drive timeout reached')
                break
            time.sleep(self._dt)

        self.stop()
        final = self.get_position()
        if final is not None and start_pos is not None:
            actual = np.linalg.norm(final[:2] - start_pos[:2])
            self.node.get_logger().info(
                f'Drive complete. Requested: {distance_m:.3f}m, '
                f'Actual: {actual:.3f}m')
        else:
            self.node.get_logger().info('Drive complete')
        return True

    def turn_angle(self, angle_deg: float, speed_dps: float = 30.0,
                   closed_loop: bool = False) -> bool:
        """Turn by *angle_deg* degrees at *speed_dps* degrees/sec.

        Positive = left (CCW), negative = right (CW).
        """
        speed_rps = min(math.radians(abs(speed_dps)), self.max_ang)
        angle_rad = math.radians(angle_deg)
        direction = 1.0 if angle_rad >= 0 else -1.0
        angle_rad = abs(angle_rad)

        if closed_loop and self.get_yaw() is not None:
            return self._turn_closed_loop(angle_rad, speed_rps, direction)
        else:
            return self._turn_open_loop(angle_rad, speed_rps, direction)

    def _turn_open_loop(self, angle_rad, speed_rps, direction):
        duration = angle_rad / max(speed_rps, 1e-6)
        self.node.get_logger().info(
            f'Turning {"left" if direction > 0 else "right"} '
            f'{math.degrees(angle_rad):.1f}° at {math.degrees(speed_rps):.1f}°/s '
            f'(open-loop, {duration:.1f}s)')

        msg = Twist()
        msg.angular.z = direction * speed_rps
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.send_offboard_heartbeat()
            self._cmd_pub.publish(msg)
            time.sleep(self._dt)

        self.stop()
        self.node.get_logger().info('Turn complete')
        return True

    def _turn_closed_loop(self, angle_rad, speed_rps, direction):
        start_yaw = self.get_yaw()
        if start_yaw is None:
            return self._turn_open_loop(angle_rad, speed_rps, direction)

        target_yaw = start_yaw + direction * angle_rad
        self.node.get_logger().info(
            f'Turning {"left" if direction > 0 else "right"} '
            f'{math.degrees(angle_rad):.1f}° (closed-loop)')

        timeout = (angle_rad / max(speed_rps, 1e-6)) * 3.0 + 5.0
        start_time = time.monotonic()
        msg = Twist()

        while True:
            cur_yaw = self.get_yaw()
            if cur_yaw is not None:
                diff = target_yaw - cur_yaw
                # Normalise to [-pi, pi]
                diff = math.atan2(math.sin(diff), math.cos(diff))
                if abs(diff) <= self.ang_tol:
                    break
                w = direction * min(speed_rps, max(0.1, abs(diff)))
                w = math.copysign(abs(w), diff)
            else:
                w = direction * speed_rps

            msg.angular.z = w
            self.send_offboard_heartbeat()
            self._cmd_pub.publish(msg)

            if time.monotonic() - start_time > timeout:
                self.node.get_logger().warn('Turn timeout reached')
                break
            time.sleep(self._dt)

        self.stop()
        self.node.get_logger().info('Turn complete')
        return True

    def drive_velocity(self, linear_x: float, angular_z: float = 0.0,
                       duration: float = 0.0):
        """Publish constant velocity for *duration* seconds (0 = single pulse)."""
        linear_x = np.clip(linear_x, -self.max_lin, self.max_lin)
        angular_z = np.clip(angular_z, -self.max_ang, self.max_ang)
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        if duration <= 0:
            self.send_offboard_heartbeat()
            self._cmd_pub.publish(msg)
            return

        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.send_offboard_heartbeat()
            self._cmd_pub.publish(msg)
            time.sleep(self._dt)
        self.stop()


# ======================================================================
# Interactive CLI node
# ======================================================================

class RobotDriverCLI(Node):
    """Standalone node with an interactive terminal for robot debugging."""

    def __init__(self):
        super().__init__('robot_driver_cli')
        self.declare_parameter('config_path', '')

        cp = self.get_parameter('config_path').get_parameter_value().string_value
        self.cfg = load_config(cp if cp else None)
        self.driver = RobotDriver(self, self.cfg)

        # Offboard heartbeat timer (keeps PX4 happy while we're in offboard)
        self._hb_timer = self.create_timer(0.05, self.driver.send_offboard_heartbeat)

    def run_cli(self):
        HELP = """
========================================
  Docking Test Suite - Robot Driver CLI
========================================
Commands:
  f <dist_m> <speed_m/s>   Drive forward
  b <dist_m> <speed_m/s>   Drive backward
  l <angle_deg> [speed_d/s] Turn left
  r <angle_deg> [speed_d/s] Turn right
  v <lin_x> <ang_z> <dur_s> Velocity command
  s                         Emergency stop
  arm                       Arm motors
  disarm                    Disarm motors
  offboard                  Enable offboard mode
  manual                    Switch to POSCTL (manual)
  pos                       Print current position
  diag                      Run diagnostics
  h                         Show this help
  q                         Quit
"""
        print(HELP)
        while rclpy.ok():
            try:
                line = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            try:
                if cmd == 'q':
                    break
                elif cmd == 'h':
                    print(HELP)
                elif cmd == 's':
                    self.driver.stop()
                    print('STOPPED')
                elif cmd == 'arm':
                    self.driver.arm()
                elif cmd == 'disarm':
                    self.driver.disarm()
                elif cmd == 'offboard':
                    self.driver.set_offboard(True)
                elif cmd == 'manual':
                    self.driver.set_offboard(False)
                elif cmd == 'pos':
                    p = self.driver.get_position()
                    y = self.driver.get_yaw()
                    if p is not None:
                        print(f'  Position: x={p[0]:.3f}  y={p[1]:.3f}  z={p[2]:.3f}')
                    else:
                        print('  Position: unavailable')
                    if y is not None:
                        print(f'  Yaw: {math.degrees(y):.1f}°')
                    else:
                        print('  Yaw: unavailable')
                elif cmd == 'f':
                    d = float(parts[1])
                    sp = float(parts[2]) if len(parts) > 2 else 0.2
                    cl = self.driver.get_position() is not None
                    self.driver.drive_distance(d, sp, closed_loop=cl)
                elif cmd == 'b':
                    d = float(parts[1])
                    sp = float(parts[2]) if len(parts) > 2 else 0.2
                    cl = self.driver.get_position() is not None
                    self.driver.drive_distance(-d, sp, closed_loop=cl)
                elif cmd == 'l':
                    a = float(parts[1])
                    sp = float(parts[2]) if len(parts) > 2 else 30.0
                    cl = self.driver.get_yaw() is not None
                    self.driver.turn_angle(a, sp, closed_loop=cl)
                elif cmd == 'r':
                    a = float(parts[1])
                    sp = float(parts[2]) if len(parts) > 2 else 30.0
                    cl = self.driver.get_yaw() is not None
                    self.driver.turn_angle(-a, sp, closed_loop=cl)
                elif cmd == 'v':
                    lx = float(parts[1])
                    az = float(parts[2]) if len(parts) > 2 else 0.0
                    dur = float(parts[3]) if len(parts) > 3 else 1.0
                    self.driver.drive_velocity(lx, az, dur)
                elif cmd == 'diag':
                    self._run_diagnostics()
                else:
                    print(f'Unknown command: {cmd}. Type "h" for help.')
            except (IndexError, ValueError) as e:
                print(f'Error: {e}. Type "h" for help.')
            except Exception as e:
                print(f'Exception: {e}')

    def _run_diagnostics(self):
        """Check which topics are active."""
        print('\n--- System Diagnostics ---')
        topic_list = self.get_topic_names_and_types()
        topic_names = {name for name, _ in topic_list}

        checks = [
            ('Vicon Robot', self.cfg['topics']['vicon_robot_pose']),
            ('Vicon Target', self.cfg['topics']['vicon_target_pose']),
            ('AprilTag Det.', self.cfg['topics']['apriltag_detections']),
            ('Camera Info', self.cfg['topics']['camera_info']),
            ('Compressed Img', self.cfg['topics'].get('camera_compressed', '')),
            ('PX4 Odometry', self.cfg['topics'].get('px4_odometry', '')),
            ('Cmd Vel', self.cfg['topics']['cmd_vel']),
        ]

        for label, topic in checks:
            if not topic:
                status = 'SKIPPED (not configured)'
            elif topic in topic_names:
                status = 'OK (topic exists)'
            else:
                status = 'MISSING'
            print(f'  {label:20s} {topic:50s} {status}')

        p = self.driver.get_position()
        print(f'\n  Position feedback: {"available" if p is not None else "NOT available"}')
        print('--- End Diagnostics ---\n')


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverCLI()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    cli_thread = threading.Thread(target=node.run_cli, daemon=True)
    cli_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
