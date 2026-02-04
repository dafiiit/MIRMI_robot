import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrackedObject:
    def __init__(self, obj_id, centroid, max_history=60):
        self.id = obj_id
        self.centroid = centroid # (x, y)
        self.history = [] # list of (timestamp, brightness)
        self.max_history = max_history
        self.last_seen_time = 0.0
        self.valid_frequency = False
        self.measured_frequency = 0.0

    def update(self, centroid, brightness, timestamp):
        self.centroid = centroid
        self.last_seen_time = timestamp
        self.history.append((timestamp, brightness))
        if len(self.history) > self.max_history:
            self.history.pop(0)

    def analyze_frequency(self, target_freq, tolerance, min_samples=30):
        if len(self.history) < min_samples:
            self.valid_frequency = False
            return False

        # Extract signal
        times = np.array([t for t, b in self.history])
        signal = np.array([b for t, b in self.history])
        
        # Remove DC component
        signal = signal - np.mean(signal)
        
        if np.std(signal) < 5: # Signal too weak/flat
            self.valid_frequency = False
            return False

        # Estimate sampling rate
        avg_dt = np.mean(np.diff(times))
        if avg_dt <= 0: return False
        fs = 1.0 / avg_dt

        # FFT
        n = len(signal)
        freqs = np.fft.fftfreq(n, d=avg_dt)
        fft_vals = np.abs(np.fft.fft(signal))
        
        # Ignore negative freqs and 0 Hz
        mask = (freqs > 0)
        freqs = freqs[mask]
        fft_vals = fft_vals[mask]

        if len(fft_vals) == 0:
            return False

        peak_idx = np.argmax(fft_vals)
        peak_freq = freqs[peak_idx]
        self.measured_frequency = peak_freq
        
        if abs(peak_freq - target_freq) <= tolerance:
            self.valid_frequency = True
            return True
        else:
            self.valid_frequency = False
            return False

class IRTrackerNode(Node):
    def __init__(self):
        super().__init__('ir_tracker_node')

        # Parameters
        self.declare_parameter('threshold', 200) # Higher default for bright spots
        self.declare_parameter('target_frequency', 10.0) # Hz
        self.declare_parameter('frequency_tolerance', 2.0) # Hz
        self.declare_parameter('history_size', 60) # frames
        
        self.threshold_val = self.get_parameter('threshold').value
        self.target_freq = self.get_parameter('target_frequency').value
        self.freq_tolerance = self.get_parameter('frequency_tolerance').value
        self.history_size = self.get_parameter('history_size').value
        
        # Subscribers
        self.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', self.infra_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/infra1/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/ir_led/pose', 10)
        self.debug_pub = self.create_publisher(Image, '/ir_led/debug_view', 10)
        self.status_pub = self.create_publisher(Point, '/ir_led/status_debug', 10)
        self.is_tracked_pub = self.create_publisher(Bool, '/ir_led/is_tracked', 10)

        self.bridge = CvBridge()
        self.depth_frame = None
        self.camera_intrinsics = None
        
        # Tracking state
        self.tracked_objects = {} # id -> TrackedObject
        self.next_obj_id = 0
        self.max_dist_threshold = 20.0 # pixels, max movement between frames

        self.get_logger().info('IR Tracker Node Started with Frequency Detection.')
        self.get_logger().info(f'Target Freq: {self.target_freq} Hz, Tolerance: {self.freq_tolerance}')

    def info_callback(self, msg):
        self.camera_intrinsics = msg

    def depth_callback(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def infra_callback(self, msg):
        # Update params
        self.threshold_val = self.get_parameter('threshold').value
        self.target_freq = self.get_parameter('target_frequency').value
        self.freq_tolerance = self.get_parameter('frequency_tolerance').value
        
        try:
            current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().error(f'Infra callback error: {e}')
            return

        # 1. Threshold
        _, mask = cv2.threshold(current_frame, self.threshold_val, 255, cv2.THRESH_BINARY)
        
        # 2. Find Connected Components (centroids)
        # connectivity=8, return type=CV_32S
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)
        
        # 3. Match with existing tracked objects
        current_candidates = []
        for i in range(1, num_labels): # Skip 0 (background)
            # Area filter
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 3 or area > 1000: # Tune these?
                continue
                
            cx, cy = centroids[i]
            # Get max brightness in this component (or avg)
            # simpler to just check brightness at centroid or max in ROI
            # Let's take the pixel value at integer centroid
            ix, iy = int(cx), int(cy)
            b = 0
            if 0 <= iy < current_frame.shape[0] and 0 <= ix < current_frame.shape[1]:
                b = current_frame[iy, ix]
                
            current_candidates.append({'id': -1, 'centroid': (cx, cy), 'brightness': b})

        # Simple tracking: Match to closest
        # Mark all existing as not updated
        active_ids = []
        
        for cand in current_candidates:
            cx, cy = cand['centroid']
            best_id = -1
            min_dist = self.max_dist_threshold
            
            # Find closest existing object
            for obj_id, obj in self.tracked_objects.items():
                ox, oy = obj.centroid
                dist = np.hypot(cx - ox, cy - oy)
                if dist < min_dist:
                    min_dist = dist
                    best_id = obj_id
            
            if best_id != -1:
                # Update existing
                self.tracked_objects[best_id].update((cx, cy), cand['brightness'], timestamp)
                active_ids.append(best_id)
            else:
                # Create new
                new_obj = TrackedObject(self.next_obj_id, (cx, cy), self.history_size)
                new_obj.update((cx, cy), cand['brightness'], timestamp)
                self.tracked_objects[self.next_obj_id] = new_obj
                active_ids.append(self.next_obj_id)
                self.next_obj_id += 1
        
        current_time = timestamp
        
        matched_ids = set(active_ids)
        keys_to_remove = []
        
        for obj_id, obj in self.tracked_objects.items():
            if obj_id not in matched_ids:
                if current_time - obj.last_seen_time > 1.0: # Lost for 1 second
                    keys_to_remove.append(obj_id)
                else:
                    # Append 0 brightness (or low value)
                    # Keep same centroid
                    obj.update(obj.centroid, 0, current_time) 
                    pass
        
        for k in keys_to_remove:
            del self.tracked_objects[k]

        # 4. Frequency Analysis & Best Candidate Selection
        tracking_candidate = None
        
        for obj_id, obj in self.tracked_objects.items():
            is_freq_match = obj.analyze_frequency(self.target_freq, self.freq_tolerance)
            if is_freq_match:
                tracking_candidate = obj
                break 

        is_tracked = False
        if tracking_candidate:
            is_tracked = True
            # Publish Pose
            cx, cy = tracking_candidate.centroid
            
            if self.depth_frame is not None and self.camera_intrinsics is not None:
                h, w = self.depth_frame.shape
                # Ensure integer coords for lookup
                ix, iy = int(max(0, min(w-1, cx))), int(max(0, min(h-1, cy)))
                depth_val = self.depth_frame[iy, ix]
                depth_m = depth_val * 0.001
                if depth_m > 0:
                    point_3d = self.project_pixel_to_3d(cx, cy, depth_m)
                    self.publish_pose(point_3d, msg.header)

        # Publish status
        msg_tracked = Bool()
        msg_tracked.data = is_tracked
        self.is_tracked_pub.publish(msg_tracked)

        # Debug View
        if self.debug_pub.get_subscription_count() > 0:
            debug_img = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR)
            
            for obj_id, obj in self.tracked_objects.items():
                cx, cy = int(obj.centroid[0]), int(obj.centroid[1])
                
                color = (0, 0, 255) # Red = noise
                if obj.valid_frequency:
                    color = (0, 255, 0) # Green = match
                
                # Draw history trace or ID
                cv2.circle(debug_img, (cx, cy), 5, color, 1)
                cv2.putText(debug_img, f"{obj.measured_frequency:.1f}Hz", (cx+5, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def project_pixel_to_3d(self, u, v, z):
        if self.camera_intrinsics is None:
            return None
        
        fx = self.camera_intrinsics.k[0]
        fy = self.camera_intrinsics.k[4]
        cx = self.camera_intrinsics.k[2]
        cy = self.camera_intrinsics.k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return Point(x=x, y=y, z=z)

    def publish_pose(self, point, header):
        if point is None: return
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position = point
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IRTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
