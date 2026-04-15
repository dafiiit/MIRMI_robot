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

        # Keep both trigger QoS variants so CLI (RELIABLE default) and
        # Foxglove (often BEST_EFFORT) can both trigger start/stop topics.
        # A BEST_EFFORT subscription is compatible with both BEST_EFFORT and
        # RELIABLE publishers, while avoiding RELIABILITY mismatch warnings.
        trigger_qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Clock topic publisher (10 Hz = 0.1 second interval)
        self.clock_publisher = self.create_publisher(Clock, '/clock', 10)
        self.create_timer(0.1, self._publish_clock)

        # Define the components and their commands
        self.components = {
            'station_detection_APRILTAG': ['ros2', 'launch', 'station_detection_APRILTAG', 'APRILTAG_detection.launch.py'],
            'station_detection_LIDAR': ['bash', '-c', 'source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash && source ~/ws_sensor_combined/install/setup.bash && ros2 launch station_detection_LIDAR LIDAR_detection.launch.py start_livox_driver:=false'],
            'station_detection_IR': ['ros2', 'launch', 'station_detection_IR', 'tracker.launch.py'],
            'pole_detector': ['ros2', 'launch', 'pole_detector', 'pole_detector.launch.py'],
            'offboard': ['ros2', 'service', 'call', '/px4/offboard', 'std_srvs/srv/SetBool', '{data: true}'],
            'arming': ['ros2', 'service', 'call', '/px4/arm', 'std_srvs/srv/SetBool', '{data: true}'],
            'camera_stream': ['bash', '-c', "gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=960, height=480, framerate=24/1' ! queue ! nvvidconv ! 'video/x-raw(memory:NVMM), format=NV12' ! queue ! nvv4l2h264enc bitrate=1000000 profile=2 preset-level=1 iframeinterval=30 idrinterval=30 insert-sps-pps=1 ! queue ! rtph264pay config-interval=1 pt=96 ! queue ! udpsink host=Rosamo port=5000"]
        }

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
