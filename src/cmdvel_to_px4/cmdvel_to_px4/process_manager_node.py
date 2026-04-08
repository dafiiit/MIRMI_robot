import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import os
import signal

class ProcessManagerNode(Node):
    def __init__(self):
        super().__init__('process_manager_node')

        self.processes = {}

        # Define the components and their commands
        self.components = {
            'station_detection_APRILTAG': ['ros2', 'launch', 'station_detection_APRILTAG', 'APRILTAG_detection.launch.py'],
            'station_detection_LIDAR': ['ros2', 'launch', 'station_detection_LIDAR', 'station_detector.launch.py'],
            'station_detection_IR': ['ros2', 'launch', 'station_detection_IR', 'tracker.launch.py'],
            'pole_detector': ['ros2', 'launch', 'pole_detector', 'pole_detector.launch.py'],
            'video_streamer': ['ros2', 'run', 'video_streamer', 'video_stream'],
            'offboard': ['ros2', 'service', 'call', '/px4/offboard', 'std_srvs/srv/SetBool', '{data: true}'],
            'arming': ['ros2', 'service', 'call', '/px4/arm', 'std_srvs/srv/SetBool', '{data: true}']
        }

        self.subscribers_start = []
        self.subscribers_stop = []

        for name in self.components.keys():
            self.processes[name] = None
            
            # Create start subscriber
            sub_start = self.create_subscription(
                Empty,
                f'/start_{name}',
                lambda msg, n=name: self.start_callback(n),
                10
            )
            self.subscribers_start.append(sub_start)

            # Create stop subscriber
            sub_stop = self.create_subscription(
                Empty,
                f'/stop_{name}',
                lambda msg, n=name: self.stop_callback(n),
                10
            )
            self.subscribers_stop.append(sub_stop)

            self.get_logger().info(f"Configured start/stop triggers for {name}")

    def start_callback(self, name):
        if self.processes[name] is None or self.processes[name].poll() is not None:
            self.get_logger().info(f"Starting {name}...")
            cmd = self.components[name]
            # Use preexec_fn=os.setsid to create a new process group, so that we can kill all child processes like launch files
            self.processes[name] = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.get_logger().info(f"Started {name} with PID {self.processes[name].pid}")
        else:
            self.get_logger().warn(f"{name} is already running!")

    def stop_callback(self, name):
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
