import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Empty
from rosgraph_msgs.msg import Clock
import subprocess
import os
import signal
import threading

class ProcessManagerNode(Node):
    def __init__(self):
        super().__init__('process_manager_node')

        self.processes = {}
        self._last_trigger_ns = {}
        self._trigger_debounce_ns = 300_000_000  # 300 ms

        # Use BEST_EFFORT for trigger topics to match Foxglove and avoid
        # RELIABILITY mismatch warnings in mixed tooling setups.
        trigger_qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Clock topic publisher (10 Hz = 0.1 second interval)
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self.create_timer(0.1, self._publish_clock)

        # LIDAR preflight for robust start behavior.
        # - Ensures a host IP exists on the expected interface.
        # - Verifies LiDAR reachability before launching detector stack.
        # Override defaults with env vars: LIVOX_IFACE, LIVOX_HOST_IP, LIVOX_LIDAR_IP.
        lidar_start_cmd = (
            'set -eo pipefail; '
            'LIVOX_IFACE="${LIVOX_IFACE:-enP8p1s0}"; '
            'LIVOX_HOST_IP="${LIVOX_HOST_IP:-192.168.1.50/24}"; '
            'LIVOX_LIDAR_IP="${LIVOX_LIDAR_IP:-192.168.1.145}"; '
            'if ! ip -4 addr show dev "$LIVOX_IFACE" | grep -q "${LIVOX_HOST_IP%/*}/"; then '
            'echo "[LIDAR preflight] adding $LIVOX_HOST_IP on $LIVOX_IFACE"; '
            'if sudo -n true >/dev/null 2>&1; then '
            'sudo -n ip addr add "$LIVOX_HOST_IP" dev "$LIVOX_IFACE"; '
            'else '
            'echo "[LIDAR preflight] missing passwordless sudo for IP recovery. Run netplan setup once (see setup_lidar_netplan.sh)."; '
            'exit 2; '
            'fi; '
            'fi; '
            'if ! ping -c 1 -W 1 "$LIVOX_LIDAR_IP" >/dev/null 2>&1; then '
            'echo "[LIDAR preflight] cannot reach $LIVOX_LIDAR_IP from $LIVOX_IFACE"; '
            'exit 1; '
            'fi; '
            'source /opt/ros/humble/setup.bash; '
            'source ~/ws_livox/install/setup.bash; '
            'source ~/ws_sensor_combined/install/setup.bash; '
            'ros2 launch station_detection_LIDAR LIDAR_detection.launch.py start_livox_driver:=true'
        )

        # Define the components and their commands
        self.components = {
            'station_detection_APRILTAG': [
                'ros2', 'launch', 'station_detection_APRILTAG', 'APRILTAG_detection.launch.py'
            ],
            'station_detection_LIDAR': [
                'bash', '-lc', lidar_start_cmd
            ],
            'station_detection_IR': [
                'ros2', 'launch', 'station_detection_IR', 'tracker.launch.py'
            ],
            'pole_detector': [
                'ros2', 'launch', 'pole_detector', 'pole_detector.launch.py'
            ],
            'offboard': [
                'ros2', 'service', 'call', '/px4/offboard',
                'std_srvs/srv/SetBool', '{data: true}'
            ],
            'arming': [
                'ros2', 'service', 'call', '/px4/arm',
                'std_srvs/srv/SetBool', '{data: true}'
            ],
            'camera_stream': [
                'bash', '-c',
                "gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! "
                "'video/x-raw(memory:NVMM), width=960, height=480, framerate=24/1' ! "
                "queue ! nvvidconv ! "
                "'video/x-raw(memory:NVMM), format=NV12' ! "
                "queue ! nvv4l2h264enc bitrate=1000000 profile=2 preset-level=1 "
                "iframeinterval=30 idrinterval=30 insert-sps-pps=1 ! "
                "queue ! rtph264pay config-interval=1 pt=96 ! "
                "queue ! udpsink host=Rosamo port=5000"
            ],

            'pointcloud_to_laserscan': [
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source ~/ws_sensor_combined/install/setup.bash && '
                'ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args '
                '-p target_frame:=livox_frame '
                '-p min_height:=0.1 '
                '-p max_height:=1.0 '
                '-p range_min:=0.1 '
                '--remap cloud_in:=/livox/lidar '
                '--remap scan:=/scan'
            ],

            'vicon_bridge': [
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source ~/ws_sensor_combined/install/setup.bash && '
                'ros2 launch vicon_bridge all_segments.launch.py'
            ],

            'ekf': [
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source ~/ws_sensor_combined/install/setup.bash && '
                'cd ~/robot_localization && '
                'ros2 launch ekf_launch.py'
            ],

            'static_tf_livox': [
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source ~/ws_sensor_combined/install/setup.bash && '
                'ros2 run tf2_ros static_transform_publisher '
                '--x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 '
                '--frame-id base_link_ekf --child-frame-id livox_frame'
            ],
            'px4_odom_bridge': [
                'bash', '-lc',
                'source /opt/ros/humble/setup.bash && '
                'source ~/ws_sensor_combined/install/setup.bash && '
                'ros2 run px4_odom_bridge px4_odom_bridge'
            ],
        }
        #'-p range_max:=4.0 '
        self.subscribers_start = []
        self.subscribers_stop = []

        for name in self.components.keys():
            self.processes[name] = None
            
            # Keep BEST_EFFORT for all triggers (Foxglove compatibility).
            self.subscribers_start.append(
                self.create_subscription(
                    Empty,
                    f'/start_{name}',
                    lambda msg, n=name: self.start_callback(n),
                    trigger_qos_best_effort,
                )
            )
            self.subscribers_stop.append(
                self.create_subscription(
                    Empty,
                    f'/stop_{name}',
                    lambda msg, n=name: self.stop_callback(n),
                    trigger_qos_best_effort,
                )
            )

            self.get_logger().info(f"Configured start/stop triggers for {name}")

    def _publish_clock(self):
        """Publish the current time to the /clock topic."""
        clock_msg = Clock()
        clock_msg.clock = self.get_clock().now().to_msg()
        self.clock_publisher.publish(clock_msg)

    def _log_subprocess_output(self, name, pipe, stream_name):
        """Read from a subprocess pipe and log lines to ROS logger."""
        try:
            for line in iter(pipe.readline, b''):
                decoded = line.decode('utf-8', errors='replace').rstrip('\n')
                if decoded:
                    self.get_logger().info(f"[{name}/{stream_name}] {decoded}")
        except Exception as e:
            self.get_logger().error(f"Error reading {stream_name} from {name}: {e}")
        finally:
            pipe.close()

    def start_callback(self, name):
        if self._is_duplicate_trigger('start', name):
            return

        if name == 'station_detection_LIDAR' and self._station_lidar_running_anywhere():
            self.get_logger().warn(
                'station_detection_LIDAR appears to be already running (tracked or external). '
                'Skipping duplicate start.'
            )
            return

        if self.processes[name] is None or self.processes[name].poll() is not None:
            self.get_logger().info(f"Starting {name}...")
            cmd = self.components[name]
            # Use preexec_fn=os.setsid to create a new process group, so that we can kill all child processes like launch files
            self.processes[name] = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            # Spawn threads to read stdout and stderr and log them to rosout
            stdout_thread = threading.Thread(
                target=self._log_subprocess_output,
                args=(name, self.processes[name].stdout, "stdout"),
                daemon=True
            )
            stderr_thread = threading.Thread(
                target=self._log_subprocess_output,
                args=(name, self.processes[name].stderr, "stderr"),
                daemon=True
            )
            stdout_thread.start()
            stderr_thread.start()
            self.get_logger().info(f"Started {name} with PID {self.processes[name].pid}")
        else:
            self.get_logger().warn(f"{name} is already running!")

    def stop_callback(self, name):
        if self._is_duplicate_trigger('stop', name):
            return
        if self.processes[name] is not None and self.processes[name].poll() is None:
            self.get_logger().info(f"Stopping {name}...")
            # Kill the entire process group to ensure children are cleanly killed
            try:
                os.killpg(os.getpgid(self.processes[name].pid), signal.SIGINT)
                self.processes[name].wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Timeout expired for {name}. Using SIGKILL.")
                os.killpg(os.getpgid(self.processes[name].pid), signal.SIGKILL)
            except Exception as e:
                self.get_logger().error(f"Error stopping {name}: {e}")
            finally:
                self.processes[name] = None
                self.get_logger().info(f"Stopped {name}.")
        elif name == 'station_detection_LIDAR':
            # Fallback for cases where LIDAR stack was started by a different
            # process_manager instance or manual terminal launch.
            if self._stop_station_lidar_orphans():
                self.get_logger().info(
                    'Stopped station_detection_LIDAR external/orphan processes.'
                )
            else:
                self.get_logger().warn(f"{name} is not currently running.")
        else:
            self.get_logger().warn(f"{name} is not currently running.")

    def _is_duplicate_trigger(self, action: str, name: str) -> bool:
        key = (action, name)
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._last_trigger_ns.get(key)
        if last_ns is not None and (now_ns - last_ns) < self._trigger_debounce_ns:
            return True
        self._last_trigger_ns[key] = now_ns
        return False

    def _station_lidar_running_anywhere(self) -> bool:
        patterns = [
            'ros2 launch station_detection_LIDAR LIDAR_detection.launch.py',
            'livox_ros_driver2_node',
            'station_detection_LIDAR/station_detector',
        ]
        for pattern in patterns:
            result = subprocess.run(
                ['pgrep', '-f', pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if result.returncode == 0:
                return True
        return False

    def _stop_station_lidar_orphans(self) -> bool:
        patterns = [
            'ros2 launch station_detection_LIDAR LIDAR_detection.launch.py',
            'livox_ros_driver2_node',
            'station_detection_LIDAR/station_detector',
        ]
        was_running = False
        for pattern in patterns:
            result = subprocess.run(
                ['pgrep', '-f', pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if result.returncode == 0:
                was_running = True
                subprocess.run(
                    ['pkill', '-f', pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
        return was_running

def main(args=None):
    rclpy.init(args=args)
    node = ProcessManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Process manager stopped via KeyboardInterrupt.")
    finally:
        # Cleanup all running processes before shutdown
        for name in node.components.keys():
            node.stop_callback(name)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
